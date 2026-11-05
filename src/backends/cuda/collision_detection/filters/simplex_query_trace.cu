#include <collision_detection/filters/simplex_query_trace.h>
#include <cuda_tool/cuda_tool.h>
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <type_traits>

namespace uipc::backend::cuda::simplex_query_trace
{
namespace
{
    using Word = std::uint64_t;
    constexpr std::array<char, 8> Magic = {'U', 'I', 'P', 'C', 'B', 'Q', '0', '1'};
    constexpr std::array<char, 8> End = {'B', 'Q', 'E', 'N', 'D', '0', '0', '1'};
    constexpr Word Version     = 1;
    constexpr Word HeaderBytes = 8 + 4 * sizeof(Word);
    constexpr Word FooterBytes = 8 + 2 * sizeof(Word);
    // Bound corrupt record allocations independently of the whole trace size.
    constexpr Word         MaxMetadataBytes = Word{256} * 1024 * 1024;
    constexpr std::uint8_t MetadataTag = 1, RequestTag = 2;

    [[noreturn]] void fail(const char* message)
    {
        throw std::runtime_error(std::string{"Simplex query trace: "} + message);
    }
    void write_bytes(std::ostream& out, const void* data, Word n)
    {
        if(n > static_cast<Word>(std::numeric_limits<std::streamsize>::max()))
            fail("record exceeds stream limits");
        if(n)
            out.write(static_cast<const char*>(data), static_cast<std::streamsize>(n));
        if(!out)
            fail("write failed");
    }
    template <typename T>
    void write_scalar(std::ostream& out, const T& value)
    {
        write_bytes(out, &value, sizeof(T));
    }
    struct Decoder
    {
        std::istream& in;
        Word          remaining;
        void          bytes(void* data, Word n)
        {
            if(n > remaining
               || n > static_cast<Word>(std::numeric_limits<std::streamsize>::max()))
                fail("invalid or truncated record length");
            if(n)
                in.read(static_cast<char*>(data), static_cast<std::streamsize>(n));
            if(!in)
                fail("truncated record");
            remaining -= n;
        }
        template <typename T>
        T scalar()
        {
            T value;
            bytes(&value, sizeof(T));
            return value;
        }
    };
    template <typename T>
    void write_vector(std::ostream& out, const std::vector<T>& values)
    {
        write_scalar(out, Word{values.size()});
        if constexpr(std::is_arithmetic_v<T>)
            write_bytes(out, values.data(), values.size() * sizeof(T));
        else
        {
            using Scalar          = typename T::Scalar;
            constexpr int       N = T::SizeAtCompileTime;
            std::vector<Scalar> packed(values.size() * N);
            for(SizeT i = 0; i < values.size(); ++i)
                std::memcpy(packed.data() + i * N, values[i].data(), N * sizeof(Scalar));
            write_bytes(out, packed.data(), packed.size() * sizeof(Scalar));
        }
    }
    template <typename T>
    void read_vector(Decoder&        input,
                     std::vector<T>& values,
                     Word            maximum = std::numeric_limits<Word>::max())
    {
        const Word     n             = input.scalar<Word>();
        constexpr Word element_bytes = []
        {
            if constexpr(std::is_arithmetic_v<T>)
                return sizeof(T);
            else
                return T::SizeAtCompileTime * sizeof(typename T::Scalar);
        }();
        if(n > maximum || n > input.remaining / element_bytes || n > values.max_size())
            fail("vector length exceeds record");
        values.resize(static_cast<SizeT>(n));
        if constexpr(std::is_arithmetic_v<T>)
            input.bytes(values.data(), n * element_bytes);
        else
        {
            using Scalar          = typename T::Scalar;
            constexpr int       N = T::SizeAtCompileTime;
            std::vector<Scalar> packed(static_cast<SizeT>(n) * N);
            input.bytes(packed.data(), n * element_bytes);
            for(SizeT i = 0; i < values.size(); ++i)
                std::memcpy(values[i].data(), packed.data() + i * N, N * sizeof(Scalar));
        }
    }
    // One inventory drives metadata encoding, decoding and exact epoch comparison.
    template <typename M>
    auto arrays(M& m)
    {
        return std::tie(m.d_hats,
                        m.thicknesses,
                        m.v2bs,
                        m.dimensions,
                        m.contact_ids,
                        m.subscene_ids,
                        m.body_self_collision,
                        m.codim_vertices,
                        m.surface_vertices,
                        m.edges,
                        m.triangles);
    }
    template <typename M>
    auto matrices(M& m)
    {
        return std::tie(m.contact_mask, m.subscene_mask);
    }
    void check_matrix(const Matrix& m)
    {
        if((m.cols && m.rows > std::numeric_limits<SizeT>::max() / m.cols)
           || m.rows > std::numeric_limits<int>::max()
           || m.cols > std::numeric_limits<int>::max()
           || m.values.size() != m.rows * m.cols)
            fail("invalid matrix shape");
    }
    std::string encode(const Metadata& m)
    {
        std::ostringstream out{std::ios::out | std::ios::binary};
        std::apply([&](const auto&... v) { (write_vector(out, v), ...); }, arrays(m));
        std::apply(
            [&](const auto&... v)
            {
                (
                    [&]
                    {
                        check_matrix(v);
                        write_scalar(out, Word{v.rows});
                        write_scalar(out, Word{v.cols});
                        write_vector(out, v.values);
                    }(),
                    ...);
            },
            matrices(m));
        return out.str();
    }
    std::shared_ptr<Metadata> decode(const std::string& bytes)
    {
        std::istringstream in{bytes, std::ios::in | std::ios::binary};
        Decoder            input{in, bytes.size()};
        auto               m = std::make_shared<Metadata>();
        std::apply([&](auto&... v) { (read_vector(input, v), ...); }, arrays(*m));
        std::apply(
            [&](auto&... v)
            {
                (
                    [&]
                    {
                        v.rows = input.scalar<Word>();
                        v.cols = input.scalar<Word>();
                        read_vector(input, v.values);
                        check_matrix(v);
                    }(),
                    ...);
            },
            matrices(*m));
        if(input.remaining)
            fail("metadata contains trailing bytes");
        return m;
    }
    void check_request(const Request& r)
    {
        if(!r.metadata || r.positions.size() != r.displacements.size()
           || r.positions.size() > static_cast<SizeT>(std::numeric_limits<IndexT>::max())
           || !std::isfinite(r.alpha) || r.alpha < 0 || !std::isfinite(r.skin))
            fail("invalid request shape or scalar");
        const auto& m = *r.metadata;
        const SizeT n = r.positions.size();
        if(m.d_hats.size() < n || m.thicknesses.size() < n || m.v2bs.size() < n
           || m.dimensions.size() < n || m.contact_ids.size() < n
           || m.subscene_ids.size() < n)
            fail("vertex metadata is shorter than positions");
        check_matrix(m.contact_mask);
        check_matrix(m.subscene_mask);
        auto vertex = [&m, n](IndexT v)
        {
            if(v < 0 || static_cast<SizeT>(v) >= n)
                fail("topology index exceeds positions");
            // Global arrays also contain implicit-geometry vertices. Only
            // simplex-referenced IDs address these predicate tables.
            const auto bid = m.v2bs[v], cid = m.contact_ids[v], sid = m.subscene_ids[v];
            if(bid < -1
               || (bid >= 0 && static_cast<SizeT>(bid) >= m.body_self_collision.size())
               || cid < 0
               || static_cast<SizeT>(cid)
                      >= std::min(m.contact_mask.rows, m.contact_mask.cols)
               || sid < 0
               || static_cast<SizeT>(sid)
                      >= std::min(m.subscene_mask.rows, m.subscene_mask.cols))
                fail("metadata index exceeds its table");
        };
        for(auto v : m.codim_vertices)
            vertex(v);
        for(auto v : m.surface_vertices)
            vertex(v);
        for(const auto& e : m.edges)
            for(int i = 0; i < 2; ++i)
                vertex(e[i]);
        for(const auto& f : m.triangles)
            for(int i = 0; i < 3; ++i)
                vertex(f[i]);
    }
    template <typename T>
    std::vector<T> download(cuda_tool::CBufferView<T> values)
    {
        std::vector<T> out(values.size());
        if(!out.empty())
            values.copy_to(out.data());
        return out;
    }
    Matrix download(cuda_tool::CBuffer2DView<IndexT> values)
    {
        const auto extent = values.extent();
        Matrix     out{extent.h, extent.w, std::vector<IndexT>(extent.count())};
        if(!out.values.empty())
            CUDA_TOOL_CHECK(cudaMemcpy(out.values.data(),
                                       values.data(),
                                       out.values.size() * sizeof(IndexT),
                                       cudaMemcpyDeviceToHost));
        return out;
    }
}  // namespace

struct Writer::Impl
{
    std::ofstream out;
    std::string   previous_metadata;
    Summary       stats;
    SizeT         last_frame = 0;
    explicit Impl(const std::filesystem::path& path)
    {
        if(std::filesystem::exists(path))
            fail("output already exists");
        if(!path.parent_path().empty())
            std::filesystem::create_directories(path.parent_path());
        out.open(path, std::ios::binary);
        if(!out)
            fail("cannot open output");
        write_bytes(out, Magic.data(), Magic.size());
        for(Word v : {Version,
                      Word{sizeof(Float)},
                      Word{sizeof(IndexT)},
                      Word{std::endian::native == std::endian::little}})
            write_scalar(out, v);
    }
};
Writer::Writer(const std::filesystem::path& path)
    : m_impl{std::make_unique<Impl>(path)}
{
}
Writer::~Writer()
{
    try
    {
        close();
    }
    catch(...)
    { /* An incomplete file is rejected by Reader. */
    }
}
void Writer::append(const Request& request)
{
    auto& s = *m_impl;
    if(s.stats.complete)
        fail("append after close");
    check_request(request);
    if(s.stats.requests && request.site.frame < s.last_frame)
        fail("physical frames went backwards");
    const auto metadata = encode(*request.metadata);
    if(metadata.size() > MaxMetadataBytes)
        fail("metadata exceeds the 256 MiB diagnostic record limit");
    if(s.stats.epochs == 0 || metadata != s.previous_metadata)
    {
        write_scalar(s.out, MetadataTag);
        write_scalar(s.out, Word{++s.stats.epochs});
        write_scalar(s.out, Word{metadata.size()});
        write_bytes(s.out, metadata.data(), metadata.size());
        s.previous_metadata = metadata;
    }
    write_scalar(s.out, RequestTag);
    write_scalar(s.out, Word{++s.stats.requests});
    write_scalar(s.out, Word{s.stats.epochs});
    write_scalar(s.out, Word{request.site.frame});
    write_scalar(s.out, Word{request.site.newton_iteration});
    write_scalar(s.out, std::uint8_t{request.new_frame});
    write_scalar(s.out, request.alpha);
    write_scalar(s.out, request.skin);
    write_vector(s.out, request.positions);
    write_vector(s.out, request.displacements);
    s.last_frame  = request.site.frame;
    s.stats.bytes = static_cast<SizeT>(s.out.tellp());
}
void Writer::capture(const SimplexTrajectoryFilter::DetectInfo& info, Site site)
{
    cuda_tool::wait_device();
    auto m                 = std::make_shared<Metadata>();
    m->d_hats              = download(info.d_hats());
    m->thicknesses         = download(info.thicknesses());
    m->v2bs                = download(info.v2b());
    m->dimensions          = download(info.dimensions());
    m->contact_ids         = download(info.contact_element_ids());
    m->subscene_ids        = download(info.subscene_element_ids());
    m->body_self_collision = download(info.body_self_collision());
    m->codim_vertices      = download(info.codim_vertices());
    m->surface_vertices    = download(info.surf_vertices());
    m->edges               = download(info.surf_edges());
    m->triangles           = download(info.surf_triangles());
    m->contact_mask        = download(info.contact_mask_tabular());
    m->subscene_mask       = download(info.subscene_mask_tabular());
    Request request;
    request.site          = site;
    request.new_frame     = !info.reuse_bvh_topology();
    request.alpha         = info.alpha();
    request.skin          = info.skin();
    request.metadata      = std::move(m);
    request.positions     = download(info.positions());
    request.displacements = download(info.displacements());
    append(request);
}
void Writer::close()
{
    auto& s = *m_impl;
    if(s.stats.complete)
        return;
    write_bytes(s.out, End.data(), End.size());
    write_scalar(s.out, Word{s.stats.requests});
    write_scalar(s.out, Word{s.stats.epochs});
    s.out.flush();
    if(!s.out)
        fail("cannot finalize output");
    s.stats.bytes = static_cast<SizeT>(s.out.tellp());
    s.out.close();
    if(s.out.fail())
        fail("cannot close output");
    s.stats.complete = true;
}
Summary Writer::summary() const
{
    return m_impl->stats;
}

struct Reader::Impl
{
    std::ifstream                   in;
    std::unique_ptr<Decoder>        input;
    std::shared_ptr<const Metadata> metadata;
    Summary                         stats;
    Word                            expected_requests = 0, expected_epochs = 0;
    SizeT                           last_frame = 0;
    explicit Impl(const std::filesystem::path& path)
    {
        const Word size = std::filesystem::file_size(path);
        if(size < HeaderBytes + FooterBytes)
            fail("incomplete header/footer");
        in.open(path, std::ios::binary);
        if(!in)
            fail("cannot open input");
        in.seekg(static_cast<std::streamoff>(size - FooterBytes));
        Decoder             footer{in, FooterBytes};
        std::array<char, 8> magic;
        footer.bytes(magic.data(), magic.size());
        if(magic != End)
            fail("missing clean footer");
        expected_requests = footer.scalar<Word>();
        expected_epochs   = footer.scalar<Word>();
        in.seekg(0);
        Decoder header{in, HeaderBytes};
        header.bytes(magic.data(), magic.size());
        if(magic != Magic || header.scalar<Word>() != Version
           || header.scalar<Word>() != sizeof(Float) || header.scalar<Word>() != sizeof(IndexT)
           || header.scalar<Word>() != Word{std::endian::native == std::endian::little})
            fail("incompatible header/version/scalar representation");
        input = std::make_unique<Decoder>(Decoder{in, size - HeaderBytes - FooterBytes});
        stats.bytes = size;
    }
};
Reader::Reader(const std::filesystem::path& path)
    : m_impl{std::make_unique<Impl>(path)}
{
}
Reader::~Reader() = default;
bool Reader::next(Request& request)
{
    auto& s     = *m_impl;
    auto& input = *s.input;
    while(input.remaining)
    {
        const auto tag = input.scalar<std::uint8_t>();
        if(tag == MetadataTag)
        {
            const auto epoch = input.scalar<Word>();
            const auto n     = input.scalar<Word>();
            if(epoch != s.stats.epochs + 1 || n > MaxMetadataBytes || n > input.remaining)
                fail("invalid metadata epoch/length");
            std::string bytes(static_cast<SizeT>(n), '\0');
            input.bytes(bytes.data(), n);
            s.metadata = decode(bytes);
            ++s.stats.epochs;
        }
        else if(tag == RequestTag)
        {
            request.id    = input.scalar<Word>();
            request.epoch = input.scalar<Word>();
            if(request.id != s.stats.requests + 1 || !s.metadata
               || request.epoch != s.stats.epochs)
                fail("invalid request sequence/epoch");
            request.site.frame            = input.scalar<Word>();
            request.site.newton_iteration = input.scalar<Word>();
            const auto new_frame          = input.scalar<std::uint8_t>();
            if(new_frame > 1 || (s.stats.requests && request.site.frame < s.last_frame))
                fail("invalid frame flag/order");
            request.new_frame    = new_frame != 0;
            request.alpha        = input.scalar<Float>();
            request.skin         = input.scalar<Float>();
            request.metadata     = s.metadata;
            const auto& m        = *s.metadata;
            const SizeT capacity = std::min({m.d_hats.size(),
                                             m.thicknesses.size(),
                                             m.v2bs.size(),
                                             m.dimensions.size(),
                                             m.contact_ids.size(),
                                             m.subscene_ids.size()});
            read_vector(input, request.positions, capacity);
            read_vector(input, request.displacements, request.positions.size());
            check_request(request);
            ++s.stats.requests;
            s.last_frame = request.site.frame;
            return true;
        }
        else
            fail("unknown record tag");
    }
    if(s.stats.requests != s.expected_requests || s.stats.epochs != s.expected_epochs)
        fail("footer counts do not match records");
    s.stats.complete = true;
    return false;
}
Summary Reader::summary() const
{
    return m_impl->stats;
}
}  // namespace uipc::backend::cuda::simplex_query_trace

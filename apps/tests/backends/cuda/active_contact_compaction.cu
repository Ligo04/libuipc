#include <collision_detection/filters/details/active_contact_compaction.h>
#include <app/app.h>
#include <algorithm>
#include <array>
#include <vector>

namespace
{
using namespace uipc;
namespace ct = uipc::backend::cuda_tool;
using uipc::backend::cuda::details::ActiveContactChunkSize;
using uipc::backend::cuda::details::ActiveContactCompaction;

template <typename T>
void require_equal(const ct::DeviceBuffer<T>& device, const std::vector<T>& expected)
{
    std::vector<T> actual;
    device.copy_to(actual);
    REQUIRE(actual.size() == expected.size());
    REQUIRE(std::equal(actual.begin(),
                       actual.end(),
                       expected.begin(),
                       [](const T& a, const T& b)
                       { return (a.array() == b.array()).all(); }));
}

struct Contacts
{
    std::vector<Vector2i> pps;
    std::vector<Vector3i> pes;
    std::vector<Vector4i> pts;
    std::vector<Vector4i> ees;
};

struct Fixture
{
    ct::DeviceBuffer<Vector2i> pps, temp_pps;
    ct::DeviceBuffer<Vector3i> pes, temp_pes;
    ct::DeviceBuffer<Vector4i> pts, temp_pts, ees, temp_ees;
    ct::DeviceBuffer<IndexT>   counts;
    ActiveContactCompaction    compact{pps, pes, pts, ees, counts};

    void append(const Contacts& input)
    {
        temp_pps.copy_from(input.pps.data(), input.pps.size());
        temp_pes.copy_from(input.pes.data(), input.pes.size());
        temp_pts.copy_from(input.pts.data(), input.pts.size());
        temp_ees.copy_from(input.ees.data(), input.ees.size());
        compact.append(
            temp_pps.cview(), temp_pes.cview(), temp_pts.cview(), temp_ees.cview());
    }

    void require_contacts(const Contacts& expected)
    {
        require_equal(pps, expected.pps);
        require_equal(pes, expected.pes);
        require_equal(pts, expected.pts);
        require_equal(ees, expected.ees);
    }

    void require_bounded(const Contacts& expected)
    {
        // DeviceBuffer grows by 1.5x. Its requested range must never exceed
        // the active prefix plus one chunk, even for mostly inactive input.
        REQUIRE(pps.capacity() <= (expected.pps.size() + ActiveContactChunkSize) * 3 / 2);
        REQUIRE(pes.capacity() <= (expected.pes.size() + ActiveContactChunkSize) * 3 / 2);
        REQUIRE(pts.capacity() <= (expected.pts.size() + ActiveContactChunkSize) * 3 / 2);
        REQUIRE(ees.capacity() <= (expected.ees.size() + ActiveContactChunkSize) * 3 / 2);
        REQUIRE(temp_pps.capacity() <= ActiveContactChunkSize * 3 / 2);
        REQUIRE(temp_pes.capacity() <= ActiveContactChunkSize * 3 / 2);
        REQUIRE(temp_pts.capacity() <= ActiveContactChunkSize * 3 / 2);
        REQUIRE(temp_ees.capacity() <= ActiveContactChunkSize * 3 / 2);
    }
};

template <typename T>
void generate(std::vector<T>& input, std::vector<T>& expected, SizeT offset, SizeT count, SizeT period, SizeT phase)
{
    input.assign(count, T::Constant(-1));
    for(SizeT i = 0; i < count; ++i)
    {
        const SizeT id = offset + i;
        if(id % period != phase % period)
            continue;
        // Consecutive duplicate contacts and an increasing identity make
        // unstable selection or lost prefixes observable independently.
        for(int j = 0; j < T::RowsAtCompileTime; ++j)
            input[i](j) = static_cast<IndexT>(id / 3 + j);
        expected.push_back(input[i]);
    }
}

Contacts append_generated(Fixture& fixture, SizeT count, SizeT period)
{
    Contacts expected;
    for(SizeT offset = 0; offset < count; offset += ActiveContactChunkSize)
    {
        Contacts    input;
        const SizeT n = std::min(ActiveContactChunkSize, count - offset);
        generate(input.pps, expected.pps, offset, n, period, 0);
        generate(input.pes, expected.pes, offset, n, period, 1);
        generate(input.pts, expected.pts, offset, n, period, 2);
        generate(input.ees, expected.ees, offset, n, period, 3);
        fixture.append(input);
    }
    return expected;
}

TEST_CASE("Active contact compaction preserves source order and duplicates",
          "[cuda][collision][active_contact_compaction]")
{
    Fixture fixture;
    fixture.compact.reset();
    fixture.append({{{8, 9}, {-1, -1}, {8, 9}, {3, 4}}, {{10, 11, 12}}, {}, {}});
    fixture.append({{}, {}, {}, {{7, 8, 9, 10}, {-1, -1, -1, -1}, {7, 8, 9, 10}}});
    fixture.append({{{-1, -1}, {-1, -1}}, {}, {}, {}});
    fixture.append({});
    fixture.append({{{1, 2}}, {{5, 6, 7}}, {{4, 5, 6, 7}}, {}});
    fixture.require_contacts({{{8, 9}, {8, 9}, {3, 4}, {1, 2}},
                              {{10, 11, 12}, {5, 6, 7}},
                              {{4, 5, 6, 7}},
                              {{7, 8, 9, 10}, {7, 8, 9, 10}}});
}

TEST_CASE("Active contact compaction handles empty and chunk boundaries",
          "[cuda][collision][active_contact_compaction]")
{
    for(SizeT count :
        {SizeT{0}, SizeT{1}, ActiveContactChunkSize - 1, ActiveContactChunkSize, ActiveContactChunkSize + 1})
    {
        CAPTURE(count);
        Fixture fixture;
        fixture.compact.reset();
        auto expected = append_generated(fixture, count, 3);
        fixture.require_contacts(expected);
        fixture.require_bounded(expected);
    }
}

TEST_CASE("Active contact compaction preserves prefixes through allocation growth",
          "[cuda][collision][active_contact_compaction]")
{
    Fixture fixture;
    fixture.compact.reset();
    auto expected = append_generated(fixture, 2 * ActiveContactChunkSize + 17, 1);
    fixture.require_contacts(expected);
    fixture.require_bounded(expected);
    REQUIRE(fixture.pps.capacity() > ActiveContactChunkSize * 3 / 2);

    auto*      pp_allocation = fixture.pps.data();
    const auto pp_capacity   = fixture.pps.capacity();
    fixture.compact.reset();
    fixture.require_contacts({});
    fixture.append({{{2, 3}, {2, 3}}, {}, {}, {}});
    fixture.append({{}, {}, {}, {{4, 5, 6, 7}}});
    fixture.require_contacts({{{2, 3}, {2, 3}}, {}, {}, {{4, 5, 6, 7}}});
    REQUIRE(fixture.pps.data() == pp_allocation);
    REQUIRE(fixture.pps.capacity() == pp_capacity);
}

TEST_CASE("Sparse million-candidate compaction bounds scratch and output capacity",
          "[cuda][collision][active_contact_compaction]")
{
    Fixture fixture;
    fixture.compact.reset();
    constexpr SizeT count = 4 * ActiveContactChunkSize + 31;
    static_assert(count > 1000000);
    auto expected = append_generated(fixture, count, 100003);
    REQUIRE(expected.pps.size() == 11);
    fixture.require_contacts(expected);
    fixture.require_bounded(expected);
    REQUIRE(fixture.pps.capacity() < count / 2);
    REQUIRE(fixture.pes.capacity() < count / 2);
    REQUIRE(fixture.pts.capacity() < count / 2);
    REQUIRE(fixture.ees.capacity() < count / 2);
}

TEST_CASE("Active contact compaction rejects unbounded chunks before allocation",
          "[cuda][collision][active_contact_compaction]")
{
    Fixture                   fixture;
    ct::CBufferView<Vector2i> oversized{nullptr, ActiveContactChunkSize + 1};
    REQUIRE_THROWS_AS(fixture.compact.append(oversized), std::length_error);
    REQUIRE(fixture.pps.capacity() == 0);
    REQUIRE(fixture.counts.capacity() == 0);
}
}  // namespace

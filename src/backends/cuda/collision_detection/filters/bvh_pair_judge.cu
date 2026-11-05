/*
 * NVIDIA contributions: trajectory BVH refit, retained query ordering and
 * corresponding reuse/invalidation logic; swept boxes with the step_forward
 * rounding, conservative FP32 distance and swept-motion rejection bounds,
 * queries from a contact-tabular vertex cover, concurrent tree chains on side
 * streams, stored-pair judging with per-group skins and FP32 inputs, detection
 * skins for candidate reuse, BVH topology retention across frames, the fused
 * detect prefix, the four-list active-set compaction and its pending active
 * set, one-warp CCD launch shapes, the in-kernel TOI minimum, TOI filtering
 * on a launch stream and batched, copy-free count read-backs; whole-warp
 * judging of the stored cover EE pairs by call site with certified FP32 bound
 * and gap screens, a one-pass judge scan with a kept-tile scatter, the cover
 * remap and the detection hook before the candidate-count read-back, and the
 * EE TOI launch beside the other TOI launches.
 *
 * The NVIDIA notice applies to the contributions described above, not to the
 * pre-existing libuipc or other contributors' code.
 */

/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <collision_detection/filters/bvh_pair_judge.h>
#include <cuda_tool/cub.h>
#include <utils/distance/swept_lower_bound_f32_certain.h>
#include <cub/block/block_reduce.cuh>
#include <cub/block/block_scan.cuh>
#include <algorithm>
namespace uipc::backend::cuda
{
namespace
{
    using namespace bvh_simplex_detail;
    using InfoStacklessBVHSimplexTrajectoryFilter_judge_ee_queue = bvh_pair_judge::EEQueue;
    using InfoStacklessBVHSimplexTrajectoryFilter_judged_list = bvh_pair_judge::PairList;
    using InfoStacklessBVHSimplexTrajectoryFilter_judged_lists = bvh_pair_judge::PairLists;
    // Judges a stored (query, leaf) pair as the traversal judges the leaf: the
    // current boxes overlap, then the leaf predicate's geometric tests. The
    // node predicate and the leaf predicates' position-free checks (contact
    // and subscene masks, shared vertices, body self-collision, the cover side
    // rule) passed when the pair was stored and depend only on ids and the
    // tabular, whose change refreshes the stored pairs; the EE and PT
    // predicates skip them (skip_position_free). One 128-pair tile per block:
    // the block also writes its kept count for compact_judged_lists. Self-join
    // pairs are (min, max) with the query arrays equal to the leaf arrays.
    constexpr int JudgeTile = bvh_pair_judge::JudgeTile;

    template <typename LeafPred>
    __global__ void __launch_bounds__(JudgeTile)
        InfoStacklessBVHSimplexTrajectoryFilter_judge_stored_kernel(
            cuda_tool::CBufferView<Vector2i> pairs,
            cuda_tool::CBufferView<AABB>     query_aabbs,
            cuda_tool::CBufferView<AABB>     leaf_aabbs,
            LeafPred                         leaf_pred,
            cuda_tool::BufferView<int>       flags,
            cuda_tool::BufferView<IndexT>    tile_counts,
            int                              n,
            const IndexT*                    invalid)
    {
        if(invalid && *invalid)
        {
            const int i = blockIdx.x * JudgeTile + threadIdx.x;
            if(i < n)
                flags(i) = 0;
            if(threadIdx.x == 0)
                tile_counts(blockIdx.x) = 0;
            return;
        }

        using BlockReduce = cub::BlockReduce<int, JudgeTile>;
        __shared__ typename BlockReduce::TempStorage storage;
        constexpr IndexT                             none = -1;

        int i    = blockIdx.x * JudgeTile + threadIdx.x;
        int keep = 0;
        if(i < n)
        {
            Vector2i p = pairs(i);
            keep       = leaf_aabbs(p[1]).intersects(query_aabbs(p[0]))
                   && leaf_pred(InfoStacklessBVH::LeafPredInfo{p[0], p[1], none, none, none, none});
            flags(i) = keep;
        }
        int kept = BlockReduce(storage).Sum(keep);
        if(threadIdx.x == 0)
            tile_counts(blockIdx.x) = kept;
    }

    // leaf.intersects(query) of AlignedBox (leaf.min <= query.max, query.min
    // <= leaf.max) with all six comparisons evaluated, so that both boxes
    // load at once: [min x, min y], [min z, max x], [max y, max z].
    __device__ __forceinline__ bool judge_boxes_overlap(const AABB* leaf, const AABB* query)
    {
        static_assert(sizeof(AABB) == 6 * sizeof(float));
        const float2* l  = reinterpret_cast<const float2*>(leaf);
        const float2* q  = reinterpret_cast<const float2*>(query);
        const float2  l0 = l[0], l1 = l[1], l2 = l[2];
        const float2  q0 = q[0], q1 = q[1], q2 = q[2];
        return (l0.x <= q1.y) & (l0.y <= q2.x) & (l1.x <= q2.y) & (q0.x <= l1.y)
               & (q0.y <= l2.x) & (q1.x <= l2.y);
    }

    // The stored cover EE pairs, judged as judge_stored_kernel judges them
    // with the cover EE predicate: the box test, then the self-join predicate
    // on (min, max) of the edge ids, called (not inlined) from one of the cover
    // predicate's two call sites by the edge order, with bids and cids none.
    // That kernel runs the called predicate twice in a warp whose lanes order
    // their edges both ways, and in any warp with one overlapping pair; here a
    // tile first queues its overlapping pairs by call site, then whole warps,
    // one site each, call the same compiled predicate from the same sites.
    __global__ void __launch_bounds__(JudgeTile)
        InfoStacklessBVHSimplexTrajectoryFilter_judge_cover_ee_kernel(
            cuda_tool::CBufferView<Vector2i> pairs,
            cuda_tool::CBufferView<AABB>     query_aabbs,
            cuda_tool::CBufferView<AABB>     leaf_aabbs,
            InfoStacklessBVHSimplexTrajectoryFilter_detect_cover_AllE_AllE_pred cover,
            cuda_tool::BufferView<int>    flags,
            cuda_tool::BufferView<IndexT> tile_counts,
            int                           n,
            const IndexT*                 invalid)
    {
        if(invalid && *invalid)
        {
            const int i = blockIdx.x * JudgeTile + threadIdx.x;
            if(i < n)
                flags(i) = 0;
            if(threadIdx.x == 0)
                tile_counts(blockIdx.x) = 0;
            return;
        }

        using BlockReduce = cub::BlockReduce<int, JudgeTile>;
        __shared__ typename BlockReduce::TempStorage storage;
        __shared__ int2  queued[2][JudgeTile];  // (i, j), pair
        __shared__ int   queued_count[2];
        constexpr IndexT none = -1;

        if(threadIdx.x < 2)
            queued_count[threadIdx.x] = 0;
        __syncthreads();

        const int i = blockIdx.x * JudgeTile + static_cast<int>(threadIdx.x);
        bool      overlap = false;
        int       site    = 0;
        int       ij      = 0;  // the query edge id
        if(i < n)
        {
            const int2 p = reinterpret_cast<const int2*>(pairs.data())[i];
            overlap      = judge_boxes_overlap(leaf_aabbs.data() + p.y,
                                          query_aabbs.data() + p.x);
            if(overlap)
            {
                ij = (cover.query_edges.size() ? cover.query_edges(p.x) : p.x);
                site = ij < p.y ? 0 : 1;
            }
            else
            {
                flags(i) = 0;
            }
        }
        const unsigned lane = threadIdx.x & 31u;
#pragma unroll
        for(int c = 0; c < 2; ++c)
        {
            const bool     mine = overlap && site == c;
            const unsigned mask = __ballot_sync(0xffffffffu, mine);
            int            base = 0;
            if(lane == 0 && mask)
                base = atomicAdd(&queued_count[c], __popc(mask));
            base = __shfl_sync(0xffffffffu, base, 0);
            if(mine)
                queued[c][base + __popc(mask & ((1u << lane) - 1u))] = make_int2(ij, i);
        }
        __syncthreads();

        // detect_cover_AllE_AllE_pred with skip_position_free: the self-join
        // predicate on (i, j) if i < j, else on (j, i). Site 0's pairs take the
        // first lanes and site 1's start at the next warp, so each warp calls
        // from one site (at most two rounds).
        const int c0   = queued_count[0];
        const int r0   = (c0 + 31) / 32 * 32;
        const int end  = r0 + queued_count[1];
        int       keep = 0;
        for(int t = static_cast<int>(threadIdx.x); t < end; t += JudgeTile)
        {
            if(t < c0)
            {
                const int2     q = queued[0][t];
                const Vector2i p = pairs(q.y);
                const int      k = cover.pred(
                    InfoStacklessBVH::LeafPredInfo{q.x, p[1], none, none, none, none});
                flags(q.y) = k;
                keep += k;
            }
            else if(t >= r0)
            {
                const int2     q = queued[1][t - r0];
                const Vector2i p = pairs(q.y);
                const int      k = cover.pred(
                    InfoStacklessBVH::LeafPredInfo{p[1], q.x, none, none, none, none});
                flags(q.y) = k;
                keep += k;
            }
        }
        const int kept = BlockReduce(storage).Sum(keep);
        if(threadIdx.x == 0)
            tile_counts(blockIdx.x) = kept;
    }

    // A trajectory detect with FP32 inputs judges the stored cover EE pairs in
    // three launches that write judge_cover_ee_kernel's flags and tile counts.
    // The predicate's FP32 rejection has two tests; on the predicate's
    // arguments, judge_screen_ee_kernel (one tile per block, no call) runs each
    // pair's box test and the certain form of the first
    // (swept_lower_bound_f32_certain.h), judge_gap_ee_kernel the certain form of
    // the second on the pairs left (about one in seven of the overlapping
    // ones, in whole warps). Where a certain test holds, the predicate's test
    // rejects the pair however it is compiled. A pair that the box test or a
    // certain test rejects is not kept, flag 0; the screen zeroes the tile
    // counts, and the second pass queues the pairs it leaves by call site.
    // judge_finish_ee_kernel calls the predicate, FP32 rejection included, on
    // those from judge_cover_ee_kernel's two call sites in whole warps, writes
    // their flags and adds their keeps to the tile counts. Only integer counts
    // depend on the order of any pass.
    constexpr int JudgeGapBlocksPerSM    = 8;
    constexpr int JudgeFinishBlocksPerSM = 4;


    // The certain test of the first (gap = false) or the second FP32 rejection
    // of the predicate on the edges (E0, E1) = (min, max) of the pair's ids.
    __device__ __forceinline__ bool judge_ee_certain(const EdgeEdgePredicate& pred, int2 ids, bool gap)
    {
        const int2*   Es   = reinterpret_cast<const int2*>(pred.Es.data());
        const int2    E0   = Es[ids.x];
        const int2    E1   = Es[ids.y];
        const float4* x    = pred.f32_positions.data();
        const float4* d    = pred.f32_steps.data();
        const float   skin = static_cast<float>(pred.skin);
        return gap ?
                   distance::edge_edge_swept_farther_than_f32_certain_gap(
                       x[E0.x], x[E0.y], x[E1.x], x[E1.y], d[E0.x], d[E0.y], d[E1.x], d[E1.y], skin) :
                   distance::edge_edge_swept_farther_than_f32_certain_bound(
                       x[E0.x], x[E0.y], x[E1.x], x[E1.y], d[E0.x], d[E0.y], d[E1.x], d[E1.y], skin);
    }

    // (min, max) of a queued pair's edge ids
    __device__ __forceinline__ int2 judge_ee_ids(const int4& s)
    {
        return s.y < s.z ? make_int2(s.y, s.z) : make_int2(s.z, s.y);
    }

    // Appends the entries of a warp's lanes that pass `mine` to `out` (from the
    // back if from_back), counted at `count`.
    __device__ __forceinline__ void judge_ee_append(
        bool mine, int* count, int4* out, int4 s, int n, bool from_back)
    {
        const unsigned lane = threadIdx.x & 31u;
        const unsigned mask = __ballot_sync(0xffffffffu, mine);
        if(mask == 0)
            return;
        int base = 0;
        if(lane == 0)
            base = atomicAdd(count, __popc(mask));
        base = __shfl_sync(0xffffffffu, base, 0);
        if(mine)
        {
            const int k = base + __popc(mask & ((1u << lane) - 1u));
            out[from_back ? n - 1 - k : k] = s;
        }
    }

    __global__ void __launch_bounds__(JudgeTile)
        InfoStacklessBVHSimplexTrajectoryFilter_judge_screen_ee_kernel(
            cuda_tool::CBufferView<Vector2i> pairs,
            cuda_tool::CBufferView<AABB>     query_aabbs,
            cuda_tool::CBufferView<AABB>     leaf_aabbs,
            InfoStacklessBVHSimplexTrajectoryFilter_detect_cover_AllE_AllE_pred cover,
            cuda_tool::BufferView<int>                             flags,
            cuda_tool::BufferView<IndexT>                          tile_counts,
            InfoStacklessBVHSimplexTrajectoryFilter_judge_ee_queue queue,
            int                                                    n,
            const IndexT*                                          invalid)
    {
        if(invalid && *invalid)
        {
            const int i = blockIdx.x * JudgeTile + threadIdx.x;
            if(i < n)
                flags(i) = 0;
            if(threadIdx.x == 0)
                tile_counts(blockIdx.x) = 0;
            return;
        }

        const int i = blockIdx.x * JudgeTile + static_cast<int>(threadIdx.x);
        bool left = false;  // neither the box test nor the first certain test rejects
        int4 s = make_int4(i, 0, 0, 0);  // pair index, query edge, leaf edge
        if(i < n)
        {
            const int2 p = reinterpret_cast<const int2*>(pairs.data())[i];
            if(judge_boxes_overlap(leaf_aabbs.data() + p.y, query_aabbs.data() + p.x))
            {
                s.y = (cover.query_edges.size() ? cover.query_edges.data()[p.x] : p.x);
                s.z  = p.y;
                left = !judge_ee_certain(cover.pred, judge_ee_ids(s), false);
            }
            if(!left)
                flags(i) = 0;
        }
        if(threadIdx.x == 0)
            tile_counts(blockIdx.x) = 0;
        judge_ee_append(left, queue.counts, queue.second, s, queue.n, false);
    }

    // Whole warps run the second certain test over the pairs the screen left
    // and queue the pairs it leaves for the predicate by call site.
    __global__ void __launch_bounds__(JudgeTile)
        InfoStacklessBVHSimplexTrajectoryFilter_judge_gap_ee_kernel(
            InfoStacklessBVHSimplexTrajectoryFilter_detect_cover_AllE_AllE_pred cover,
            cuda_tool::BufferView<int>                             flags,
            InfoStacklessBVHSimplexTrajectoryFilter_judge_ee_queue queue)
    {
        const int m      = queue.counts[0];
        const int lane   = static_cast<int>(threadIdx.x & 31u);
        const int stride = static_cast<int>(gridDim.x * blockDim.x);
        for(int base = static_cast<int>(blockIdx.x * blockDim.x + (threadIdx.x & ~31u));
            base < m;
            base += stride)
        {
            const int t      = base + lane;
            bool      queued = false;
            int4      s      = make_int4(0, 0, 0, 0);
            if(t < m)
            {
                s      = queue.second[t];
                queued = !judge_ee_certain(cover.pred, judge_ee_ids(s), true);
                if(!queued)
                    flags(s.x) = 0;
            }
            judge_ee_append(
                queued && s.y < s.z, queue.counts + 1, queue.pairs, s, queue.n, false);
            judge_ee_append(
                queued && s.y >= s.z, queue.counts + 2, queue.pairs, s, queue.n, true);
        }
    }

    // Site 0's queued pairs take the first lanes and site 1's start at the
    // next warp, so each warp calls from one site.
    __global__ void __launch_bounds__(JudgeTile)
        InfoStacklessBVHSimplexTrajectoryFilter_judge_finish_ee_kernel(
            InfoStacklessBVHSimplexTrajectoryFilter_detect_cover_AllE_AllE_pred cover,
            cuda_tool::BufferView<int>                             flags,
            cuda_tool::BufferView<IndexT>                          tile_counts,
            InfoStacklessBVHSimplexTrajectoryFilter_judge_ee_queue queue)
    {
        constexpr IndexT none   = -1;
        const int        c0     = queue.counts[1];
        const int        r0     = (c0 + 31) / 32 * 32;
        const int        end    = r0 + queue.counts[2];
        const int        stride = static_cast<int>(gridDim.x * blockDim.x);
        for(int t = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x); t < end; t += stride)
        {
            int i = -1;
            int k = 0;
            if(t < c0)
            {
                const int4 s = queue.pairs[t];
                i            = s.x;
                k = cover.pred(InfoStacklessBVH::LeafPredInfo{s.y, s.z, none, none, none, none});
            }
            else if(t >= r0)
            {
                const int4 s = queue.pairs[queue.n - 1 - (t - r0)];
                i            = s.x;
                k = cover.pred(InfoStacklessBVH::LeafPredInfo{s.z, s.y, none, none, none, none});
            }
            if(i >= 0)
            {
                flags(i) = k;
                if(k)
                    atomicAdd(tile_counts.data() + i / JudgeTile, 1);
            }
        }
    }

    // compact_judged_lists(): the judged lists of one detect, written as
    // DeviceSelect::Flagged would write them. One block scans every list's
    // per-tile kept counts into tile offsets (and writes the counts), then one
    // warp per tile that keeps pairs writes them in order.
    constexpr int JudgedLists = bvh_pair_judge::JudgedLists;  // PP, PE, EE, PT, PcoverT (query_counts order)
    constexpr int JudgeScanThreads = 1024;


    __device__ int judged_list_tiles(int n)
    {
        return (n + JudgeTile - 1) / JudgeTile;
    }

    // The tiles of the five lists lie back to back, so one block scans them as
    // one array, JudgeScanItems consecutive tiles per thread (one pass covers
    // 16,384 tiles; 12,749 on X_square): a tile's offset is the sum of the
    // counts before it minus the sum before its list's first tile. Also writes
    // the list counts and the three escape flags into the query count array,
    // and zeroes the EE queue's counts for the next detect.
    constexpr int JudgeScanItems = 16;

    __global__ void __launch_bounds__(JudgeScanThreads)
        InfoStacklessBVHSimplexTrajectoryFilter_judge_scan_kernel(
            InfoStacklessBVHSimplexTrajectoryFilter_judged_lists lists,
            cuda_tool::CBufferView<IndexT>                       tile_counts,
            cuda_tool::BufferView<IndexT>                        tile_offsets,
            cuda_tool::CBufferView<IndexT>                       escaped,
            cuda_tool::BufferView<IndexT>                        query_counts,
            int* ee_queue_counts)
    {
        using BlockScan = cub::BlockScan<int, JudgeScanThreads>;
        __shared__ typename BlockScan::TempStorage storage;
        __shared__ int list_base[JudgedLists + 1];  // the sum before each list, the total

        constexpr int Last = JudgedLists - 1;
        const int     tiles =
            lists.list[Last].first_tile + judged_list_tiles(lists.list[Last].n);
        auto first_tile = [&](int l)
        { return l < JudgedLists ? lists.list[l].first_tile : tiles; };
        // the list of tile t (lists without tiles share their successor's first tile)
        auto list_of = [&](int t)
        {
            int l = Last;
            while(l > 0 && t < lists.list[l].first_tile)
                --l;
            return l;
        };

        int carry = 0;
        for(int base = 0; base < tiles; base += JudgeScanThreads * JudgeScanItems)
        {
            const int first = base + static_cast<int>(threadIdx.x) * JudgeScanItems;
            int c[JudgeScanItems];
            if(first + JudgeScanItems <= tiles)
            {
                const int4* v = reinterpret_cast<const int4*>(tile_counts.data() + first);
#pragma unroll
                for(int q = 0; q < JudgeScanItems / 4; ++q)
                {
                    const int4 w = v[q];
                    c[4 * q]     = w.x;
                    c[4 * q + 1] = w.y;
                    c[4 * q + 2] = w.z;
                    c[4 * q + 3] = w.w;
                }
            }
            else
            {
#pragma unroll
                for(int k = 0; k < JudgeScanItems; ++k)
                    c[k] = first + k < tiles ? tile_counts(first + k) : 0;
            }
            int x[JudgeScanItems];  // the sum before each tile within the thread
            int sum = 0;
#pragma unroll
            for(int k = 0; k < JudgeScanItems; ++k)
            {
                x[k] = sum;
                sum += c[k];
            }
            int before, total;
            BlockScan(storage).ExclusiveSum(sum, before, total);
            before += carry;

            // the sum before each list that starts at one of this thread's tiles
            for(int l = 0; l < JudgedLists; ++l)
            {
                const int f = first_tile(l);
                if(f < first || f >= first + JudgeScanItems || f >= tiles)
                    continue;
                int at = 0;
#pragma unroll
                for(int k = 0; k < JudgeScanItems; ++k)
                    at = first + k == f ? x[k] : at;
                list_base[l] = before + at;
            }
            __syncthreads();

            const int l0 = list_of(first);
            if(first + JudgeScanItems <= tiles && list_of(first + JudgeScanItems - 1) == l0)
            {
                const int b = before - list_base[l0];
                int4* o = reinterpret_cast<int4*>(tile_offsets.data() + first);
#pragma unroll
                for(int q = 0; q < JudgeScanItems / 4; ++q)
                    o[q] = make_int4(b + x[4 * q],
                                     b + x[4 * q + 1],
                                     b + x[4 * q + 2],
                                     b + x[4 * q + 3]);
            }
            else
            {
#pragma unroll
                for(int k = 0; k < JudgeScanItems; ++k)
                    if(first + k < tiles)
                        tile_offsets(first + k) =
                            before + x[k] - list_base[list_of(first + k)];
            }
            carry += total;
            __syncthreads();
        }

        if(threadIdx.x == 0)
        {
            for(int l = 0; l <= JudgedLists; ++l)  // lists from the end on start at the total
                if(first_tile(l) >= tiles)
                    list_base[l] = carry;
            for(int l = 0; l < JudgedLists; ++l)
            {
                const int count      = list_base[l + 1] - list_base[l];
                *lists.list[l].count = count;
                query_counts(l)      = count;
            }
            for(int f = 0; f < 3; ++f)
                query_counts(JudgedLists + f) = escaped(f);
            ee_queue_counts[0] = 0;
            ee_queue_counts[1] = 0;
            ee_queue_counts[2] = 0;
        }
    }

    // Warp w looks at tiles w, w + W, w + 2W, ... (W warps) at once and
    // writes the kept pairs of each tile that keeps any (a list keeps few) at
    // the tile's offset in index order.
    constexpr int JudgeScatterBlocksPerSM = 8;

    __global__ void __launch_bounds__(JudgeTile)
        InfoStacklessBVHSimplexTrajectoryFilter_judge_scatter_kernel(
            InfoStacklessBVHSimplexTrajectoryFilter_judged_lists lists,
            cuda_tool::CBufferView<IndexT>                       tile_counts,
            cuda_tool::CBufferView<IndexT>                       tile_offsets,
            int                                                  tiles)
    {
        constexpr int  Warps = JudgeTile / 32;
        const unsigned lane  = threadIdx.x & 31u;
        const int      W     = static_cast<int>(gridDim.x) * Warps;
        const int warp = static_cast<int>(blockIdx.x * Warps + threadIdx.x / 32);
        for(int base = 0; base < tiles; base += 32 * W)
        {
            const int t = base + warp + static_cast<int>(lane) * W;
            unsigned kept = __ballot_sync(0xffffffffu, t < tiles && tile_counts(t) != 0);
            while(kept)
            {
                const int tile = base + warp + (__ffs(kept) - 1) * W;
                kept &= kept - 1;
                int l = JudgedLists - 1;
                while(l > 0 && tile < lists.list[l].first_tile)
                    --l;
                const auto& L     = lists.list[l];
                const int   start = (tile - L.first_tile) * JudgeTile;
                int         out   = tile_offsets(tile);
#pragma unroll
                for(int k = 0; k < JudgeTile; k += 32)
                {
                    const int      i    = start + k + static_cast<int>(lane);
                    const bool     keep = i < L.n && L.flags[i] != 0;
                    const unsigned mask = __ballot_sync(0xffffffffu, keep);
                    if(keep)
                        L.out[out + __popc(mask & ((1u << lane) - 1u))] = L.pairs[i];
                    out += __popc(mask);
                }
            }
        }
    }


}  // namespace
namespace bvh_pair_judge
{
    template <typename Predicate>
    void judge_stored(cuda_tool::CBufferView<Vector2i> pairs,
                      cuda_tool::CBufferView<AABB>     queries,
                      cuda_tool::CBufferView<AABB>     leaves,
                      Predicate                        predicate,
                      cuda_tool::BufferView<int>       flags,
                      cuda_tool::BufferView<IndexT>    tile_counts,
                      int                              n,
                      cudaStream_t                     stream,
                      const IndexT*                    invalid)
    {
        if(n)
            InfoStacklessBVHSimplexTrajectoryFilter_judge_stored_kernel<Predicate>
                <<<(n + JudgeTile - 1) / JudgeTile, JudgeTile, 0, stream>>>(
                    pairs, queries, leaves, predicate, flags, tile_counts, n, invalid);
    }
    template void judge_stored<bvh_simplex_detail::PointPointPredicate>(
        cuda_tool::CBufferView<Vector2i>,
        cuda_tool::CBufferView<AABB>,
        cuda_tool::CBufferView<AABB>,
        bvh_simplex_detail::PointPointPredicate,
        cuda_tool::BufferView<int>,
        cuda_tool::BufferView<IndexT>,
        int,
        cudaStream_t,
        const IndexT*);
    template void judge_stored<bvh_simplex_detail::PointEdgePredicate>(
        cuda_tool::CBufferView<Vector2i>,
        cuda_tool::CBufferView<AABB>,
        cuda_tool::CBufferView<AABB>,
        bvh_simplex_detail::PointEdgePredicate,
        cuda_tool::BufferView<int>,
        cuda_tool::BufferView<IndexT>,
        int,
        cudaStream_t,
        const IndexT*);
    template void judge_stored<bvh_simplex_detail::EdgeEdgePredicate>(
        cuda_tool::CBufferView<Vector2i>,
        cuda_tool::CBufferView<AABB>,
        cuda_tool::CBufferView<AABB>,
        bvh_simplex_detail::EdgeEdgePredicate,
        cuda_tool::BufferView<int>,
        cuda_tool::BufferView<IndexT>,
        int,
        cudaStream_t,
        const IndexT*);
    template void judge_stored<bvh_simplex_detail::PointTrianglePredicate>(
        cuda_tool::CBufferView<Vector2i>,
        cuda_tool::CBufferView<AABB>,
        cuda_tool::CBufferView<AABB>,
        bvh_simplex_detail::PointTrianglePredicate,
        cuda_tool::BufferView<int>,
        cuda_tool::BufferView<IndexT>,
        int,
        cudaStream_t,
        const IndexT*);
    template void judge_stored<bvh_simplex_detail::InfoStacklessBVHSimplexTrajectoryFilter_detect_cover_AllP_AllT_pred>(
        cuda_tool::CBufferView<Vector2i>,
        cuda_tool::CBufferView<AABB>,
        cuda_tool::CBufferView<AABB>,
        bvh_simplex_detail::InfoStacklessBVHSimplexTrajectoryFilter_detect_cover_AllP_AllT_pred,
        cuda_tool::BufferView<int>,
        cuda_tool::BufferView<IndexT>,
        int,
        cudaStream_t,
        const IndexT*);

    void judge_ee(cuda_tool::CBufferView<Vector2i> pairs,
                  cuda_tool::CBufferView<AABB>     queries,
                  cuda_tool::CBufferView<AABB>     leaves,
                  EEPredicate                      predicate,
                  cuda_tool::BufferView<int>       flags,
                  cuda_tool::BufferView<IndexT>    tile_counts,
                  EEQueue                          queue,
                  int                              sm_count,
                  cudaStream_t                     stream,
                  const IndexT*                    invalid)
    {
        const int n = static_cast<int>(pairs.size());
        if(!n)
            return;
        const int tiles = (n + JudgeTile - 1) / JudgeTile;
        if(predicate.pred.alpha != 0.0 && predicate.pred.f32_positions.size())
        {
            CUDA_TOOL_CHECK(cudaMemsetAsync(queue.counts, 0, 3 * sizeof(int), stream));
            InfoStacklessBVHSimplexTrajectoryFilter_judge_screen_ee_kernel<<<tiles, JudgeTile, 0, stream>>>(
                pairs, queries, leaves, predicate, flags, tile_counts, queue, n, invalid);
            InfoStacklessBVHSimplexTrajectoryFilter_judge_gap_ee_kernel<<<std::min(sm_count * JudgeGapBlocksPerSM, tiles), JudgeTile, 0, stream>>>(
                predicate, flags, queue);
            InfoStacklessBVHSimplexTrajectoryFilter_judge_finish_ee_kernel<<<std::min(sm_count * JudgeFinishBlocksPerSM, tiles), JudgeTile, 0, stream>>>(
                predicate, flags, tile_counts, queue);
        }
        else
            InfoStacklessBVHSimplexTrajectoryFilter_judge_cover_ee_kernel<<<tiles, JudgeTile, 0, stream>>>(
                pairs, queries, leaves, predicate, flags, tile_counts, n, invalid);
    }
    void compact(PairLists                      lists,
                 cuda_tool::CBufferView<IndexT> tile_counts,
                 cuda_tool::BufferView<IndexT>  tile_offsets,
                 cuda_tool::CBufferView<IndexT> escaped,
                 cuda_tool::BufferView<IndexT>  counts,
                 int*                           ee_queue_counts,
                 int                            tiles,
                 int                            sm_count,
                 cudaStream_t                   stream)
    {
        InfoStacklessBVHSimplexTrajectoryFilter_judge_scan_kernel<<<1, JudgeScanThreads, 0, stream>>>(
            lists, tile_counts, tile_offsets, escaped, counts, ee_queue_counts);
        if(tiles)
        {
            const int blocks =
                std::min(sm_count * JudgeScatterBlocksPerSM,
                         (tiles + 4 * (JudgeTile / 32) - 1) / (4 * (JudgeTile / 32)));
            InfoStacklessBVHSimplexTrajectoryFilter_judge_scatter_kernel<<<blocks, JudgeTile, 0, stream>>>(
                lists, tile_counts, tile_offsets, tiles);
        }
    }
}  // namespace bvh_pair_judge
}  // namespace uipc::backend::cuda

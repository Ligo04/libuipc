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

#pragma once
#include <collision_detection/filters/simplex_candidate_predicates.cuh>

namespace uipc::backend::cuda::bvh_pair_judge
{
constexpr int JudgeTile   = 128;
constexpr int JudgedLists = 5;
struct EEQueue
{
    int4* second;
    int4* pairs;
    int*  counts;
    int   n;
};
struct PairList
{
    const Vector2i* pairs;
    const int*      flags;
    Vector2i*       out;
    int*            count;
    int             n;
    int             first_tile;
};
struct PairLists
{
    PairList list[JudgedLists];
};
using EEPredicate = bvh_simplex_detail::InfoStacklessBVHSimplexTrajectoryFilter_detect_cover_AllE_AllE_pred;

// A nonzero invalid flag suppresses all geometry reads and emits zero flags
// and tile counts. Its producer must precede these launches on their streams.
template <typename Predicate>
void judge_stored(cuda_tool::CBufferView<Vector2i> pairs,
                  cuda_tool::CBufferView<AABB>     queries,
                  cuda_tool::CBufferView<AABB>     leaves,
                  Predicate                        predicate,
                  cuda_tool::BufferView<int>       flags,
                  cuda_tool::BufferView<IndexT>    tile_counts,
                  int                              n,
                  cudaStream_t  stream  = cuda_tool::default_stream(),
                  const IndexT* invalid = nullptr);

void judge_ee(cuda_tool::CBufferView<Vector2i> pairs,
              cuda_tool::CBufferView<AABB>     queries,
              cuda_tool::CBufferView<AABB>     leaves,
              EEPredicate                      predicate,
              cuda_tool::BufferView<int>       flags,
              cuda_tool::BufferView<IndexT>    tile_counts,
              EEQueue                          queue,
              int                              sm_count,
              cudaStream_t  stream  = cuda_tool::default_stream(),
              const IndexT* invalid = nullptr);

void compact(PairLists                      lists,
             cuda_tool::CBufferView<IndexT> tile_counts,
             cuda_tool::BufferView<IndexT>  tile_offsets,
             cuda_tool::CBufferView<IndexT> escaped,
             cuda_tool::BufferView<IndexT>  counts,
             int*                           ee_queue_counts,
             int                            tiles,
             int                            sm_count,
             cudaStream_t stream = cuda_tool::default_stream());
}  // namespace uipc::backend::cuda::bvh_pair_judge

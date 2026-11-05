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
#include <uipc/common/vector.h>
#include <uipc/common/type_define.h>
#include <cstdint>
namespace uipc::backend::cuda::bvh_simplex_detail
{
inline vector<IndexT> contact_vertex_cover(const vector<IndexT>& mask, SizeT N, const IndexT* weights)
{
    vector<IndexT> in_cover(N, 1);
    vector<SizeT>  rest;
    for(SizeT c = 0; c < N; ++c)
    {
        if(weights[c] > 0 && !mask[c * N + c])
        {
            rest.push_back(c);
            in_cover[c] = 0;
        }
    }
    SizeT R    = rest.size();
    auto  edge = [&](SizeT a, SizeT b)
    { return a != b && mask[rest[a] * N + rest[b]] != 0; };

    if(R <= 20)
    {
        vector<uint32_t> adj(R, 0);
        for(SizeT a = 0; a < R; ++a)
            for(SizeT b = 0; b < R; ++b)
                if(edge(a, b))
                    adj[a] |= 1u << b;
        uint32_t best   = 0;
        int64_t  best_w = -1;
        for(uint32_t s = 0; s < (1u << R); ++s)
        {
            int64_t w     = 0;
            bool    cover = true;
            for(SizeT a = 0; a < R && cover; ++a)
            {
                if(s >> a & 1u)
                    w += weights[rest[a]];
                else
                    cover = (adj[a] & ~s) == 0;
            }
            if(cover && (best_w < 0 || w < best_w))
            {
                best   = s;
                best_w = w;
            }
        }
        for(SizeT a = 0; a < R; ++a)
            if(best >> a & 1u)
                in_cover[rest[a]] = 1;
        return in_cover;
    }

    while(true)
    {
        SizeT  best       = R;
        double best_ratio = 0.0;
        for(SizeT a = 0; a < R; ++a)
        {
            if(in_cover[rest[a]])
                continue;
            SizeT degree = 0;
            for(SizeT b = 0; b < R; ++b)
                if(!in_cover[rest[b]] && edge(a, b))
                    ++degree;
            if(degree == 0)
                continue;
            double ratio = double(weights[rest[a]]) / double(degree);
            if(best == R || ratio < best_ratio)
            {
                best       = a;
                best_ratio = ratio;
            }
        }
        if(best == R)
            break;
        in_cover[rest[best]] = 1;
    }
    return in_cover;
}
}  // namespace uipc::backend::cuda::bvh_simplex_detail

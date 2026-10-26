/*
 * NVIDIA contributions: fusion of the existing PT/EE/PE/PP frictional
 * contact-energy launches, contact-type dispatch, launch-interface
 * extraction and zero-work handling.
 *
 * The per-contact energy computations are adapted from libuipc's
 * ipc_simplex_frictional_contact.cu; those pre-existing portions retain
 * their original provenance.
 *
 * The NVIDIA notice below applies to the modifications described above, not
 * to the pre-existing libuipc code.
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

#include <contact_system/contact_models/ipc_simplex_frictional_contact_energy.h>

#include <contact_system/contact_models/codim_ipc_simplex_frictional_contact_function.h>
#include <utils/codim_thickness.h>
#include <utils/primitive_d_hat.h>
#include <utils/contact_type_block_layout.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void ipc_simplex_frictional_contact_energy_kernel(
        cuda_tool::CDense2D<ContactCoeff> table,
        cuda_tool::CBufferView<IndexT>    contact_ids,
        cuda_tool::CBufferView<Vector3>   Ps,
        cuda_tool::CBufferView<Vector3>   prev_Ps,
        cuda_tool::CBufferView<Vector3>   rest_Ps,
        cuda_tool::CBufferView<Float>     thicknesses,
        cuda_tool::CBufferView<Float>     d_hats,
        cuda_tool::CBufferView<Vector4i>  PTs,
        cuda_tool::CBufferView<Vector4i>  EEs,
        cuda_tool::CBufferView<Vector3i>  PEs,
        cuda_tool::CBufferView<Vector2i>  PPs,
        cuda_tool::BufferView<Float>      PT_Es,
        cuda_tool::BufferView<Float>      EE_Es,
        cuda_tool::BufferView<Float>      PE_Es,
        cuda_tool::BufferView<Float>      PP_Es,
        Float                             eps_v,
        Float                             dt,
        IndexT                            pt_end,
        IndexT                            ee_end,
        IndexT                            pe_end,
        IndexT                            pp_end)
    {
        IndexT idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= pp_end)
            return;

        using namespace sym::codim_ipc_contact;

        if(idx < pt_end)
        {
            const IndexT i         = idx;
            const auto&  PT        = PTs(i);
            Vector4i     cids      = {contact_ids(PT[0]),
                                      contact_ids(PT[1]),
                                      contact_ids(PT[2]),
                                      contact_ids(PT[3])};
            auto         coeff     = PT_contact_coeff(table, cids);
            Float        kt2       = coeff.kappa * dt * dt;
            Float        mu        = coeff.mu;
            const auto&  prev_P    = prev_Ps(PT[0]);
            const auto&  prev_T0   = prev_Ps(PT[1]);
            const auto&  prev_T1   = prev_Ps(PT[2]);
            const auto&  prev_T2   = prev_Ps(PT[3]);
            const auto&  P         = Ps(PT[0]);
            const auto&  T0        = Ps(PT[1]);
            const auto&  T1        = Ps(PT[2]);
            const auto&  T2        = Ps(PT[3]);
            Float        thickness = PT_thickness(thicknesses(PT[0]),
                                           thicknesses(PT[1]),
                                           thicknesses(PT[2]),
                                           thicknesses(PT[3]));
            Float        d_hat =
                PT_d_hat(d_hats(PT[0]), d_hats(PT[1]), d_hats(PT[2]), d_hats(PT[3]));
            PT_Es(i) = PT_friction_energy(
                kt2, d_hat, thickness, mu, eps_v * dt, prev_P, prev_T0, prev_T1, prev_T2, P, T0, T1, T2);
        }
        else if(idx < ee_end)
        {
            const IndexT   i         = idx - pt_end;
            const auto&    EE        = EEs(i);
            Vector4i       cids      = {contact_ids(EE[0]),
                                        contact_ids(EE[1]),
                                        contact_ids(EE[2]),
                                        contact_ids(EE[3])};
            auto           coeff     = EE_contact_coeff(table, cids);
            Float          kt2       = coeff.kappa * dt * dt;
            Float          mu        = coeff.mu;
            const Vector3& rest_Ea0  = rest_Ps(EE[0]);
            const Vector3& rest_Ea1  = rest_Ps(EE[1]);
            const Vector3& rest_Eb0  = rest_Ps(EE[2]);
            const Vector3& rest_Eb1  = rest_Ps(EE[3]);
            const Vector3& prev_Ea0  = prev_Ps(EE[0]);
            const Vector3& prev_Ea1  = prev_Ps(EE[1]);
            const Vector3& prev_Eb0  = prev_Ps(EE[2]);
            const Vector3& prev_Eb1  = prev_Ps(EE[3]);
            const Vector3& Ea0       = Ps(EE[0]);
            const Vector3& Ea1       = Ps(EE[1]);
            const Vector3& Eb0       = Ps(EE[2]);
            const Vector3& Eb1       = Ps(EE[3]);
            Float          thickness = EE_thickness(thicknesses(EE[0]),
                                           thicknesses(EE[1]),
                                           thicknesses(EE[2]),
                                           thicknesses(EE[3]));
            Float          d_hat =
                EE_d_hat(d_hats(EE[0]), d_hats(EE[1]), d_hats(EE[2]), d_hats(EE[3]));

            Float eps_x;
            distance::edge_edge_mollifier_threshold(
                rest_Ea0, rest_Ea1, rest_Eb0, rest_Eb1, static_cast<Float>(1e-3), eps_x);
            if(distance::need_mollify(prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, eps_x))
                EE_Es(i) = 0;
            else
                EE_Es(i) = EE_friction_energy(
                    kt2, d_hat, thickness, mu, eps_v * dt, prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, Ea0, Ea1, Eb0, Eb1);
        }
        else if(idx < pe_end)
        {
            const IndexT i  = idx - ee_end;
            const auto&  PE = PEs(i);
            Vector3i cids = {contact_ids(PE[0]), contact_ids(PE[1]), contact_ids(PE[2])};
            auto           coeff   = PE_contact_coeff(table, cids);
            Float          kt2     = coeff.kappa * dt * dt;
            Float          mu      = coeff.mu;
            const Vector3& prev_P  = prev_Ps(PE[0]);
            const Vector3& prev_E0 = prev_Ps(PE[1]);
            const Vector3& prev_E1 = prev_Ps(PE[2]);
            const Vector3& P       = Ps(PE[0]);
            const Vector3& E0      = Ps(PE[1]);
            const Vector3& E1      = Ps(PE[2]);
            Float          thickness =
                PE_thickness(thicknesses(PE[0]), thicknesses(PE[1]), thicknesses(PE[2]));
            Float d_hat = PE_d_hat(d_hats(PE[0]), d_hats(PE[1]), d_hats(PE[2]));
            PE_Es(i)    = PE_friction_energy(
                kt2, d_hat, thickness, mu, eps_v * dt, prev_P, prev_E0, prev_E1, P, E0, E1);
        }
        else
        {
            const IndexT   i       = idx - pe_end;
            const auto&    PP      = PPs(i);
            Vector2i       cids    = {contact_ids(PP[0]), contact_ids(PP[1])};
            auto           coeff   = PP_contact_coeff(table, cids);
            Float          kt2     = coeff.kappa * dt * dt;
            Float          mu      = coeff.mu;
            const Vector3& prev_P0 = prev_Ps(PP[0]);
            const Vector3& prev_P1 = prev_Ps(PP[1]);
            const Vector3& P0      = Ps(PP[0]);
            const Vector3& P1      = Ps(PP[1]);
            Float thickness = PP_thickness(thicknesses(PP[0]), thicknesses(PP[1]));
            Float d_hat = PP_d_hat(d_hats(PP[0]), d_hats(PP[1]));
            PP_Es(i)    = PP_friction_energy(
                kt2, d_hat, thickness, mu, eps_v * dt, prev_P0, prev_P1, P0, P1);
        }
    }
}  // namespace

void launch_ipc_simplex_frictional_contact_energy(const IPCSimplexFrictionalContactEnergyLaunchInfo& info)
{
    const auto layout = make_contact_type_contiguous_layout<IndexT>(
        info.PTs.size(), info.EEs.size(), info.PEs.size(), info.PPs.size());
    const IndexT pt_end = layout.pt_end;
    const IndexT ee_end = layout.ee_end;
    const IndexT pe_end = layout.pe_end;
    const IndexT pp_end = layout.pp_end;
    if(pp_end == 0)
        return;

    constexpr int BlockSize = 64;
    ipc_simplex_frictional_contact_energy_kernel<<<pp_end / BlockSize + (pp_end % BlockSize != 0), BlockSize, 0, nullptr>>>(
        info.contact_tabular.viewer(),
        info.contact_element_ids,
        info.positions,
        info.prev_positions,
        info.rest_positions,
        info.thicknesses,
        info.d_hats,
        info.PTs,
        info.EEs,
        info.PEs,
        info.PPs,
        info.PT_energies,
        info.EE_energies,
        info.PE_energies,
        info.PP_energies,
        info.eps_velocity,
        info.dt,
        pt_end,
        ee_end,
        pe_end,
        pp_end);
}
}  // namespace uipc::backend::cuda

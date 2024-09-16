#pragma once
#include <sim_system.h>
#include <global_geometry/simplicial_surface_reporter.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/affine_body_vertex_reporter.h>

namespace uipc::backend::cuda
{
class AffinebodySurfaceReporter : public SimplicialSurfaceReporter
{
  public:
    using SimplicialSurfaceReporter::SimplicialSurfaceReporter;

    class Impl;

    class Impl
    {
      public:
        void init_surface(backend::WorldVisitor& world);

        void report_count(backend::WorldVisitor& world,
                          GlobalSimpicialSurfaceManager::SurfaceCountInfo& info);
        void report_attributes(backend::WorldVisitor& world,
                               GlobalSimpicialSurfaceManager::SurfaceAttributeInfo& info);

        AffineBodyDynamics*       affine_body_dynamics = nullptr;
        AffineBodyDynamics::Impl& abd() { return affine_body_dynamics->m_impl; }
        AffineBodyVertexReporter* affine_body_vertex_reporter = nullptr;

        vector<IndexT>   surf_vertices;
        vector<Vector2i> surf_edges;
        vector<Vector3i> surf_triangles;
    };

  protected:
    virtual void do_build() override;
    virtual void do_report_count(GlobalSimpicialSurfaceManager::SurfaceCountInfo& info) override;
    virtual void do_report_attributes(GlobalSimpicialSurfaceManager::SurfaceAttributeInfo& info) override;

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda

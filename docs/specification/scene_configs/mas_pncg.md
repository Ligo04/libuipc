# MAS-PNCG

This native MAS-PNCG variant is an optional nonlinear solver in the CUDA backend,
with global CCD and Armijo acceptance. Select it through the same Scene
configuration mechanism used for choosing a Newton linear solver.

=== "C++"

    ```cpp
    auto config = uipc::core::Scene::default_config();
    config["nonlinear_system"]["solver"] = "mas_pncg";
    uipc::core::Scene scene{config};
    ```

=== "Python"

    ```python
    from uipc import Scene

    config = Scene.default_config()
    config["nonlinear_system"]["solver"] = "mas_pncg"
    scene = Scene(config)
    ```

Use `"newton"` to return to the existing Newton path. Its
`config["linear_system"]["solver"]` continues to select `"fused_pcg"` or
`"linear_pcg"`. MAS-PNCG always uses MAS and does not execute PCG. Geometry,
constitutions, contact configuration and `Engine`/`World` usage are unchanged.
All parameters, units and effective-value rules are in the
[configuration reference](../scene_config.md#nonlinear-solver-selection).

## Numerical operations

A restart captures the native projected sparse Hessian and an immutable MAS
epoch. Level-0 matrices and inverse blocks use double; coarse inverse data
retains the existing float storage with validation. Ordinary iterations evaluate
the complete current native gradient without rebuilding the global Hessian.

Normal-contact curvature updates use the native barrier, thickness, activation
distance, coefficient and time-step scaling. Contact keys retain primitive
roles and occurrence multiplicity. Updates are reconstructed against the
restart baseline: new contacts add full current curvature, stable stiffening
adds a positive difference, softening adds none, and only baseline curvature
persists after removal. Friction gradients remain current; this is not an
incremental tangential-Hessian method. Mollified EE updates request a full
native rebuild rather than substituting a different contact model.
Normal signs are equivalent for the outer product. Stability compares the
normalized geometric normals and each normalized feature weight separately.

Each domain selects up to `top_k` columns for a Woodbury correction. The complete
Level-0 result is evaluated in double, and an implicit local-operator residual
detects cancellation. A direct local solve is attempted when needed and checked
against that same operator. The global 2D curvature retains untruncated sparse
columns and base-Hessian products; MAS is not assumed to be the inverse of that
global surrogate.

Finite positive-pivot checks select the 2D minimizer or a 1D fallback. Every
direction must descend. Native feasible-step filtering and global CCD bound
the trajectory, then the complete native energy must satisfy Armijo decrease.
The solver never uses a small preconditioned step alone to declare convergence.

For free vertex $i$, let $g_i$ be the current native gradient, $m_i$ its fixed
kinetic mass and $L_i$ its owning mesh's rest extent. The local residual is
$\eta_i=\|g_i\|/(m_i L_i)$ and must satisfy
$\eta_i\le\max(\mathtt{gradient\_atol},\mathtt{gradient\_rtol}\,\eta_{i,0})$.
The global residual $r=\sqrt{\sum_i \|g_i\|^2/m_i}$ must also be below
$\max(\mathtt{gradient\_atol}\sqrt{\sum_i m_i}L,
\mathtt{gradient\_rtol}\,r_0)$, where $L$ is the free rest-scene extent.
Zero extents use 1 m. Fixed vertices do not contribute. References reset once
per frame or changed animation objective, and remain fixed across numerical
restarts. The maximum kinetic correction $\|g_i\|/m_i$ must additionally be at
most `velocity_tol * dt`, and animation targets must be reached.

## Scope and diagnostics

The supported configuration/model set is enforced at initialization. In
particular, affine bodies and rigid-soft coupling require a separate
generalized-coordinate implementation. Selecting this mode does not turn a
stiff tetrahedron into an ABD body or enable a strict SE(3) integrator.

`Engine::frame_stats()` / `engine.frame_stats()` reports
`nonlinear_solver="mas_pncg"`, nonlinear direction attempts, accepted steps,
base rebuilds, restart count, 1D/2D steps, local truncation/direct fallbacks,
rejected trials and independent residuals. Newton/PCG iteration counts remain
zero. `completed` and `converged` retain their separate meanings. Compare modes
at matching residuals and native objectives; these operations do not establish
a performance advantage over Newton on an arbitrary scene.

`normal_contacts` is the last exported native contact count, including native
occurrence multiplicity. `contact_update_columns` sums positive columns used
by global curvature products across direction attempts in the frame. It is
zero for frozen/full-rebuild modes. `nonfinite_trials` counts the nonfinite-energy
subset of `rejected_trials`. A failed frame invalidates the World and
requires a fresh Engine/World from a valid state or checkpoint; public
`World::recover()` only operates on a valid World.

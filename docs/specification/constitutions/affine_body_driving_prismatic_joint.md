# Affine Body Driving Prismatic Joint

References:

[A unified newton barrier method for multibody dynamics](https://dl.acm.org/doi/pdf/10.1145/3528223.3530076)

## #21 AffineBodyDrivingPrismaticJoint

The **Affine Body Driving Prismatic Joint** is a constraint constitution that drives a [Prismatic Joint](./affine_body_prismatic_joint.md) (UID=20) to a target distance along its sliding axis. It must be applied to a geometry that already has an `AffineBodyPrismaticJoint` constitution.

The driving joint supports two operating modes:

- **Active mode** (`is_passive = 0`): the joint drives toward a user-specified `aim_distance`.
- **Passive mode** (`is_passive = 1`): the joint resists external forces by treating the current distance as the target, effectively locking the joint in place.

The constraint can also be toggled on and off at runtime via the `driving/is_constrained` flag.

## Energy

We assume 2 affine body indices $i$ and $j$, each with their own state vector $\mathbf{q}_i$ and $\mathbf{q}_j$ as defined in the [Affine Body](./affine_body.md) constitution.

The joint axis is defined by the per-body tangent directions $\hat{\mathbf{t}}_i, \hat{\mathbf{t}}_j$ and anchor positions $\mathbf{c}_i, \mathbf{c}_j$ as in the [Prismatic Joint](./affine_body_prismatic_joint.md).

`aim_distance` is a **user-facing target** expressed in the same frame as the reported `distance` (i.e. $d_{\text{current}}$). To translate it into the raw displacement space actually used by the energy, the solver subtracts $d_{\text{init}}$ (the base joint's `init_distance`):

$$
\tilde{d} =
\begin{cases}
d_{\text{aim}} - d_{\text{init}}, & \text{is\_passive} = 0 \\
d_{\text{current}} - d_{\text{init}}, & \text{is\_passive} = 1
\end{cases}
$$

In active mode, $\tilde d$ is the raw-displacement counterpart of `aim_distance`. In passive mode, `aim_distance` is silently replaced by the current reported distance, which gives $\tilde d \approx d(\mathbf{q})$ and therefore a near-zero energy — the joint locks at the instant state and resists further motion.

The energy function is a symmetric two-body quadratic penalty on the raw signed axis projection:

$$
E = \frac{K}{2} \left( \hat{\mathbf{t}}_i \cdot (\mathbf{c}_j - \mathbf{c}_i) - \tilde{d} \right)^2
  + \frac{K}{2} \left( \hat{\mathbf{t}}_j \cdot (\mathbf{c}_j - \mathbf{c}_i) - \tilde{d} \right)^2,
$$

where $K = \gamma (m_i + m_j)$, $\gamma$ is the **user defined** `driving/strength_ratio` parameter, and $m_i$, $m_j$ are the masses of the two affine bodies. The two terms evaluate the constraint from each body's local tangent; when the base prismatic-joint constraint is satisfied the two tangents coincide and both terms reduce to $(d(\mathbf{q}) - \tilde d)^2$, i.e. a single penalty on the raw axis displacement. Substituting the definitions, the penalty is equivalent to pulling the reported `distance` toward `aim_distance` in the user-facing frame.

When `driving/is_constrained = 0`, the energy is zero and the driving effect is disabled.

The current distance $d_{\text{current}}$ and the initial offset $d_{\text{init}}$ are read from the **base** [AffineBodyPrismaticJoint](./affine_body_prismatic_joint.md), which tracks them on the `distance` / `init_distance` edge attributes and keeps them in sync with the body state. The driving joint only consumes these values — it does not own them.

## Requirement

This constitution must be applied to a geometry that already has an [AffineBodyPrismaticJoint](./affine_body_prismatic_joint.md) (UID=20) constitution.

## Attributes

On the joint geometry (1D simplicial complex), on **edges** (one edge per joint). The edge inherits all linking and state fields of the base [Affine Body Prismatic Joint](./affine_body_prismatic_joint.md): `l_geo_id`, `r_geo_id`, `l_inst_id`, `r_inst_id`, `strength_ratio`, `distance`, `init_distance`, and optional `l_position0`, `l_position1`, `r_position0`, `r_position1` when created via Local `create_geometry`.

Driving-specific attributes on **edges**:

- `driving/strength_ratio`: $\gamma$ in $K = \gamma(m_i + m_j)$ above
- `driving/is_constrained`: enables (`1`) or disables (`0`) the driving effect
- `is_passive`: passive mode (`1`) locks at the current distance; active mode (`0`) drives to `aim_distance`
- `aim_distance`: $d_{\text{aim}}$, the target distance in active mode

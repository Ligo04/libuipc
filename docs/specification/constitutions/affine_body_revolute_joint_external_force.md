# Affine Body Revolute Joint External Force

## #668 AffineBodyRevoluteJointExternalForce

The **Affine Body Revolute Joint External Force** applies a scalar external torque around the axis of a [Revolute Joint](./affine_body_revolute_joint.md). The torque drives rotational motion between the two connected affine bodies about the shared joint axis.

## Torque Application

Given a scalar torque $\tau$ and the joint axis direction $\hat{\mathbf{e}}$, the external torque is converted into a translational force at each body's center of mass. The torque produces equal-and-opposite tangential forces on the two bodies:

$$
\mathbf{F}_i = \begin{bmatrix} -\tau \, \dfrac{\hat{\mathbf{e}}_i \times \mathbf{r}_i}{\|\mathbf{r}_i\|^2} \\ \mathbf{0}_9 \end{bmatrix} \in \mathbb{R}^{12}, \quad
\mathbf{F}_j = \begin{bmatrix} +\tau \, \dfrac{\hat{\mathbf{e}}_j \times \mathbf{r}_j}{\|\mathbf{r}_j\|^2} \\ \mathbf{0}_9 \end{bmatrix} \in \mathbb{R}^{12},
$$

where:

- $\tau$ is the scalar torque magnitude (per edge)
- $\hat{\mathbf{e}}_k$ is the normalized joint axis direction in body $k$'s current frame
- $\mathbf{r}_k$ is the lever arm vector from the joint axis to body $k$'s center of mass, perpendicular to the axis
- $\mathbf{0}_9$ denotes the nine-dimensional zero vector (no affine force component)

### Axis Direction

The joint axis is defined by two points $\mathbf{x}^0$ and $\mathbf{x}^1$ on each body, following the [Revolute Joint](./affine_body_revolute_joint.md) convention. The axis direction in world space is:

$$
\hat{\mathbf{e}}_k = \frac{\mathring{\mathbf{J}}^{\hat{e}}_k \mathbf{q}_k}{\|\mathring{\mathbf{J}}^{\hat{e}}_k \mathbf{q}_k\|}, \quad k \in \{i, j\},
$$

where $\mathring{\mathbf{J}}^{\hat{e}}_k$ maps the rest-space axis direction through the current affine transform.

<a id="parent-vs-child-axis-and-positive-torque"></a>

### Parent vs. child axis and positive torque

Indexing matches the base joint: **parent** $=$ left $(i)$, **child** $=$ right $(j)$. For each joint, the CUDA backend builds two normalized axis vectors from rest segments $(\bar{\mathbf{x}}^1-\bar{\mathbf{x}}^0)$ **per body**; they coincide as a geometric line but point **opposite** directions,

$$
\hat{\mathbf{e}}_i \approx -\,\hat{\mathbf{e}}_j \qquad\text{(parent axis opposite child axis)}.
$$

The scalar attribute `external_torque` is **child-positive**: a **positive** $\tau$ applies a right-hand torque about **the child’s** axis direction $\hat{\mathbf{e}}_{\mathrm{child}} = \hat{\mathbf{e}}_j$ (equivalently, about $-\hat{\mathbf{e}}_i$ on the parent). The force kernel uses each body’s own $\hat{\mathbf{e}}_k$ and lever arm $\mathbf{r}_k$ so the pair is consistent with this convention.

This parent/child polarity matters for interpreting **only** this external-force constitution (UID 668), not for renaming $\theta$ or for limit/driving targets, which follow the shared `angle` frame in [Angle State](./affine_body_revolute_joint.md#angle-state).

### Center of mass (reference configuration)

For each body $k \in \{i, j\}$, let $\bar{\mathbf{c}}_k \in \mathbb{R}^3$ be the center of mass in the **reference configuration** — the same frame as the rest vertices and $m\bar{\mathbf{x}}$ in the dyadic mass on the affine body geometry ([Affine Body](./affine_body.md)). It is

$$
\bar{\mathbf{c}}_k = \frac{(m\bar{\mathbf{x}})_k}{m_k},
$$

using that body’s $m_k$, $(m\bar{\mathbf{x}})_k$ from `abd_mass` / `abd_mass_x_bar` (equivalently `mass_center` in meta).

### Lever Arm

Let $\bar{\mathbf{x}}^0_k$ be the rest-frame anchor point on body $k$ for the joint (paired with the axis segment as in [Revolute Joint](./affine_body_revolute_joint.md)). Write $\mathbf{T}_k(\bar{\mathbf{x}})$ for the world-space position of a rest point under body $k$’s current affine coordinates $\mathbf{q}_k$. The vector from the anchor to the center of mass in world space is

$$
\mathbf{d}_k = \mathbf{T}_k(\bar{\mathbf{c}}_k) - \mathbf{T}_k(\bar{\mathbf{x}}^0_k),
$$

equivalently the linear map applied to $\bar{\mathbf{c}}_k - \bar{\mathbf{x}}^0_k$ (translation drops out along this displacement). The lever arm orthogonal to the axis is

$$
\mathbf{r}_k = \mathbf{d}_k - (\mathbf{d}_k \cdot \hat{\mathbf{e}}_k) \hat{\mathbf{e}}_k.
$$

If $\bar{\mathbf{c}}_k = \mathbf{0}$ (reference origin at COM), $\mathbf{d}_k = -\mathbf{T}_k(\bar{\mathbf{x}}^0_k) = \mathbf{T}_k(\mathbf{0}) - \mathbf{T}_k(\bar{\mathbf{x}}^0_k)$, recovering the chord from anchor to origin.

When $\|\mathbf{r}_k\|^2 < \epsilon$ (within a numerical cutoff), that body contributes no coupling force from this step.

### Sign Convention

Let $\hat{\mathbf{e}}_{\mathrm{child}}=\hat{\mathbf{e}}_j$ and $\hat{\mathbf{e}}_{\mathrm{parent}}=\hat{\mathbf{e}}_i=-\hat{\mathbf{e}}_j$. A positive $\tau$ applies a right-hand torque about $+\hat{\mathbf{e}}_{\mathrm{child}}$ (child-positive; see [above](#parent-vs-child-axis-and-positive-torque)). The two bodies receive tangential impulse at their centers of mass via the per-body $\hat{\mathbf{e}}_k\times\mathbf{r}_k$ terms in § [Torque Application](#torque-application).

## Energy Integration

The external forces are incorporated into each affine body's kinetic energy term through the predicted position $\tilde{\mathbf{q}}$, following the same mechanism as [AffineBodyExternalForce](./affine_body_external_force.md):

$$
E = \frac{1}{2} \left(\mathbf{q} - \tilde{\mathbf{q}}\right)^T \mathbf{M} \left(\mathbf{q} - \tilde{\mathbf{q}}\right),
$$

where $\tilde{\mathbf{q}}$ is updated each time step to include the acceleration from the external force:

$$
\mathbf{a}_{ext} = \mathbf{M}^{-1} \mathbf{F}_{ext}.
$$

## State Update

The current joint angle $\theta_{\text{current}}$ is tracked and written back to the `angle` edge attribute by the **base** [AffineBodyRevoluteJoint](./affine_body_revolute_joint.md) — not by this external-force constitution. See [Angle State](./affine_body_revolute_joint.md#angle-state) for the formulation.

## Runtime Control

The torque can be updated at each frame through the Animator system:

- Set `external_torque/is_constrained` to `1` to enable the torque, or `0` to disable it.
- Modify the `external_torque` attribute to change the torque magnitude.

## Requirement

This constitution must be applied to a geometry that already has an [AffineBodyRevoluteJoint](./affine_body_revolute_joint.md) (UID=18) constitution.

## Attributes

On the joint geometry (1D simplicial complex), on **edges** (one edge per joint). The edge inherits all linking and state fields of the base [Affine Body Revolute Joint](./affine_body_revolute_joint.md): `l_geo_id`, `r_geo_id`, `l_inst_id`, `r_inst_id`, `strength_ratio`, `angle`, `init_angle`, and optional `l_position0`, `l_position1`, `r_position0`, `r_position1` when created via Local `create_geometry`.

External-force-specific attributes on **edges**:

- `external_torque`: $\tau$ in the formulae above, scalar torque around the joint axis (one per edge)
- `external_torque/is_constrained`: enables (`1`) or disables (`0`) the external torque

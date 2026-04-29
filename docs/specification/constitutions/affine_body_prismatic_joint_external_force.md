# Affine Body Prismatic Joint External Force

## #667 AffineBodyPrismaticJointExternalForce

The **Affine Body Prismatic Joint External Force** applies a scalar external force along the axis of a [Prismatic Joint](./affine_body_prismatic_joint.md). The force drives translational motion between the two connected affine bodies along the joint's tangent direction $\hat{\mathbf{t}}$.

## Force Application

Given a scalar force $f$ and the per-body joint tangent $\hat{\mathbf{t}}_k$, the external force enters as a translational generalized force on each body's affine origin DOF:

$$
\mathbf{F}_k = \begin{bmatrix} +f \, \hat{\mathbf{t}}_k \\ \mathbf{0}_9 \end{bmatrix} \in \mathbb{R}^{12}, \quad k \in \{i, j\},
$$

where:

- $f$ is the scalar force magnitude (per edge)
- $\hat{\mathbf{t}}_k$ is the joint tangent direction on body $k$ in world space (see [Tangent Direction](#tangent-direction))
- $\mathbf{0}_9$ is the nine-dimensional zero vector (no rotational generalized force)

The same $+f$ is applied to both bodies; Newton's third law is encoded by the strict antisymmetry $\hat{\mathbf{t}}_i = -\hat{\mathbf{t}}_j$ that is enforced numerically in § [Tangent Direction](#tangent-direction).

### Tangent Direction

The joint tangent on body $k$ is defined by two anchor points $\mathbf{x}^0_k$ and $\mathbf{x}^1_k$ in world space, following the base [Prismatic Joint](./affine_body_prismatic_joint.md) convention. Each body first computes its own raw world-space tangent from its own anchors and current pose:

$$
\tilde{\mathbf{t}}_k = \frac{\mathbf{x}^1_k - \mathbf{x}^0_k}{\bigl\|\mathbf{x}^1_k - \mathbf{x}^0_k\bigr\|}, \quad k \in \{i, j\}.
$$

By construction the two anchor pairs are laid out in opposite order on the two bodies, so $\tilde{\mathbf{t}}_i \approx -\tilde{\mathbf{t}}_j$ whenever the joint constraint is satisfied. To absorb residual numerical drift near the singularity (and to guarantee Newton's third law bit-exactly regardless of the per-body roundoff), the two raw tangents are symmetrized into one strictly antisymmetric pair:

$$
\hat{\mathbf{t}}_j = \tfrac{1}{2}\bigl(\tilde{\mathbf{t}}_j + \tilde{\mathbf{t}}_i\bigr),
$$

$$
\hat{\mathbf{t}}_i = -\hat{\mathbf{t}}_j.
$$

The averaging step is a first-order denoiser: as long as the joint is well-posed, $\tilde{\mathbf{t}}_i + \tilde{\mathbf{t}}_j$ is roundoff-level and the corrected tangents lie within the same $\mathcal{O}(\varepsilon)$ neighborhood as the raw ones, while strict antisymmetry $\hat{\mathbf{t}}_i = -\hat{\mathbf{t}}_j$ now holds exactly. If the magnitude of $\tilde{\mathbf{t}}_i + \tilde{\mathbf{t}}_j$ is non-trivial, the joint constraint itself has drifted and the upstream constraint solver — not this symmetrization — is responsible for the fix.

<a id="parent-vs-child-tangent-and-positive-force"></a>

### Parent vs. child tangent and positive force

**Parent** $=$ left $(i)$, **child** $=$ right $(j)$, matching the base joint. The two anchor pairs are laid out in opposite order on the two bodies, so the raw per-body tangents point opposite ways: $\tilde{\mathbf{t}}_i \approx -\tilde{\mathbf{t}}_j$. After the symmetrization in § [Tangent Direction](#tangent-direction) this becomes the exact identity $\hat{\mathbf{t}}_i = -\hat{\mathbf{t}}_j$.

A positive `external_force` applies $+f\,\hat{\mathbf{t}}_i$ to the parent and $+f\,\hat{\mathbf{t}}_j$ to the child; because $\hat{\mathbf{t}}_i = -\hat{\mathbf{t}}_j$, this drives the two bodies apart along the joint axis (or together when $f < 0$). This polarity applies only to this constitution (UID 667); the shared `distance` frame for limits and driving targets follows the base [Prismatic Joint](./affine_body_prismatic_joint.md#distance-state).

## Time Integration

`external_force` contributes **no potential energy of its own**. The generalized force $\mathbf{F}_k$ from § [Force Application](#force-application) enters the time step only through the predicted position $\tilde{\mathbf{q}}_k$ (i.e. as an inertial shift), in the same way as [AffineBodyExternalForce](./affine_body_external_force.md):

$$
\mathbf{a}_{\text{ext},k} = \mathbf{M}_k^{-1} \mathbf{F}_k.
$$

## Runtime Control

The force can be updated at each frame through the Animator system:

- Set `external_force/is_constrained` to `1` to enable the force, or `0` to disable it.
- Modify the `external_force` attribute to change the force magnitude.

## Requirement

This constitution must be applied to a geometry that already has an [AffineBodyPrismaticJoint](./affine_body_prismatic_joint.md) (UID=20) constitution.

## Attributes

On the joint geometry (1D simplicial complex), on **edges** (one edge per joint). Linking and state fields are inherited from the base [Affine Body Prismatic Joint](./affine_body_prismatic_joint.md). The fields owned by this constitution are:

- `external_force`: $f$ in the formulae above, scalar force along the joint axis (one per edge)
- `external_force/is_constrained`: enables (`1`) or disables (`0`) the external force

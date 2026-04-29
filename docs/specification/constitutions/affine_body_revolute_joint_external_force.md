# Affine Body Revolute Joint External Force

## #668 AffineBodyRevoluteJointExternalForce

The **Affine Body Revolute Joint External Force** applies a scalar external torque around the axis of a [Revolute Joint](./affine_body_revolute_joint.md). The torque drives rotational motion between the two connected affine bodies about the shared joint axis.

## Torque Application

Given a scalar torque $\tau$ and the joint axis direction $\hat{\mathbf{e}}$, the external torque is mapped via the principle of virtual work directly to an equivalent **generalized force on the affine rotational DOF** of each body — without forming a translational force or evaluating a center-of-mass lever arm. For each body $k \in \{i, j\}$ with affine matrix $\mathbf{A}_k \in \mathbb{R}^{3 \times 3}$ extracted from $\mathbf{q}_k$:

$$
\mathbf{F}_k = \begin{bmatrix} \mathbf{0}_3 \\ \operatorname{vec}\!\bigl(\mathbf{F}^A_k\bigr) \end{bmatrix} \in \mathbb{R}^{12},
\qquad
\mathbf{F}^A_k = \frac{\tau}{2} \, [\hat{\mathbf{e}}_k]_\times \, \mathbf{A}_k^{-T},
$$

where:

- $\tau$ is the scalar torque magnitude (per edge)
- $\hat{\mathbf{e}}_k$ is the normalized joint axis direction in body $k$'s current world frame
- $[\hat{\mathbf{e}}_k]_\times \in \mathbb{R}^{3 \times 3}$ is the skew-symmetric matrix satisfying $[\hat{\mathbf{e}}_k]_\times \mathbf{w} = \hat{\mathbf{e}}_k \times \mathbf{w}$
- $\mathbf{A}_k^{-T}$ is the inverse-transpose of body $k$'s current affine matrix
- $\operatorname{vec}(\cdot)$ packs a $3 \times 3$ matrix into the $9$ components of $\mathbf{q}_k[3{:}12]$ in **row-major** order, matching the row-major layout used for the rows of $\mathbf{A}_k$ in $\mathbf{q}_k$
- $\mathbf{0}_3$ denotes the three-dimensional zero vector — a pure torque produces no translational generalized force, regardless of whether $\mathbf{x}_k$ is the COM or a mesh anchor

The same factor $\tau$ is applied to both bodies; Newton's third law is encoded geometrically by $\hat{\mathbf{e}}_j \approx -\hat{\mathbf{e}}_i$ (see [below](#parent-vs-child-axis-and-positive-torque)), so the opposite axis direction supplies the opposite-sign reaction.

### Reduction to rigid body

When $\mathbf{A}_k \in SO(3)$ (no shear or scale, i.e. an undeformed rigid body), $\mathbf{A}_k^{-T} = \mathbf{A}_k$ and the formula reduces to the familiar rigid-body torque-on-rotation generalized force $\mathbf{F}^A_k = \tfrac{1}{2}[\hat{\mathbf{e}}_k]_\times \mathbf{A}_k$. The factor $\mathbf{A}_k^{-T}$ is the affine-body correction for shear/scale; it vanishes in the rigid limit.

### Axis Direction

The joint axis on body $k$ is defined by two anchor points $\mathbf{x}^0_k$ and $\mathbf{x}^1_k$ in world space, following the base [Revolute Joint](./affine_body_revolute_joint.md) convention. The axis direction is the normalized chord between them:

$$
\hat{\mathbf{e}}_k = \frac{\mathbf{x}^1_k - \mathbf{x}^0_k}{\bigl\|\mathbf{x}^1_k - \mathbf{x}^0_k\bigr\|}, \quad k \in \{i, j\}.
$$

<a id="parent-vs-child-axis-and-positive-torque"></a>

### Parent vs. child axis and positive torque

**Parent** $=$ left $(i)$, **child** $=$ right $(j)$, matching the base joint. The two anchor pairs are laid out in opposite order on the two bodies, so the axes point opposite ways: $\hat{\mathbf{e}}_i \approx -\hat{\mathbf{e}}_j$.

`external_torque` is **child-positive**: a positive $\tau$ applies a right-hand torque about $\hat{\mathbf{e}}_j$ (the child axis); the parent receives the equal-and-opposite reaction automatically because $\hat{\mathbf{e}}_i = -\hat{\mathbf{e}}_j$. This polarity applies only to this constitution (UID 668); the shared `angle` frame for limits and driving targets follows the base [Revolute Joint](./affine_body_revolute_joint.md#angle-state).

## Time Integration

`external_torque` contributes **no potential energy of its own**. The generalized force $\mathbf{F}_k$ from § [Torque Application](#torque-application) enters the time step only through the predicted position $\tilde{\mathbf{q}}_k$ (i.e. as an inertial shift), in the same way as [AffineBodyExternalForce](./affine_body_external_force.md):

$$
\mathbf{a}_{\text{ext},k} = \mathbf{M}_k^{-1} \mathbf{F}_k.
$$

## Runtime Control

The torque can be updated at each frame through the Animator system:

- Set `external_torque/is_constrained` to `1` to enable the torque, or `0` to disable it.
- Modify the `external_torque` attribute to change the torque magnitude.

## Requirement

This constitution must be applied to a geometry that already has an [AffineBodyRevoluteJoint](./affine_body_revolute_joint.md) (UID=18) constitution.

## Attributes

On the joint geometry (1D simplicial complex), on **edges** (one edge per joint). Linking and state fields are inherited from the base [Affine Body Revolute Joint](./affine_body_revolute_joint.md). The fields owned by this constitution are:

- `external_torque`: $\tau$ in the formulae above, scalar torque around the joint axis (one per edge)
- `external_torque/is_constrained`: enables (`1`) or disables (`0`) the external torque

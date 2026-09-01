import gc

import numpy as np
import pytest

import uipc
from uipc import Scene
from uipc.geometry import tetmesh


@pytest.mark.basic
def test_ndarray_layout_and_owner_lifetime() -> None:
    vertices = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    tetrahedra = np.array([[0, 1, 2, 3]], dtype=np.int32)
    mesh = tetmesh(vertices, tetrahedra)

    position_slot = mesh.positions()
    positions = position_slot.view()
    itemsize = positions.dtype.itemsize

    assert positions.shape == (4, 3, 1)
    assert positions.strides == (3 * itemsize, itemsize, 3 * itemsize)
    assert positions.flags.c_contiguous
    assert not positions.flags.owndata
    assert not positions.flags.writeable

    velocity_slot = mesh.vertices().create("velocity", uipc.Vector3.Zero())
    velocities = uipc.view(velocity_slot)
    velocity_alias = uipc.view(velocity_slot)
    velocities[1, :, :] = 2.0
    assert np.shares_memory(velocities, velocity_alias)
    assert np.array_equal(velocity_alias[1].reshape(3), np.full(3, 2.0))

    del mesh, position_slot, velocity_slot, velocity_alias
    gc.collect()

    # The ndarray owner handles must keep both backing allocations alive.
    assert np.array_equal(positions.reshape(4, 3), vertices)
    velocities[2, :, :] = 3.0
    assert np.array_equal(velocities[2].reshape(3), np.full(3, 3.0))


@pytest.mark.basic
def test_shared_object_identity_is_stable() -> None:
    scene = Scene()
    created = scene.objects().create("identity-probe")
    object_id = created.id()

    found = scene.objects().find(object_id)
    assert found is created

    del created
    gc.collect()

    # The scene and the surviving Python alias share one C++ object, and a
    # repeated shared_ptr conversion must reuse the same Python wrapper.
    assert scene.objects().find(object_id) is found
    assert found.name() == "identity-probe"

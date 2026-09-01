"""Static usage contract for the generated nanobind stubs."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
from typing_extensions import assert_type

from uipc._native.pyuipc import Float, ResidentThread, Vector3
from uipc._native.pyuipc.core import Object, Scene
from uipc._native.pyuipc.geometry import SimplicialComplex, tetmesh


assert_type(Float.One(), float)
assert_type(Vector3.Zero(), NDArray[np.float64])

worker = ResidentThread()
assert_type(worker.post(lambda: None), bool)
assert_type(worker.is_ready(), bool)

vertices: NDArray[np.float64] = np.zeros((4, 3), dtype=np.float64)
tetrahedra: NDArray[np.int32] = np.array([[0, 1, 2, 3]], dtype=np.int32)
mesh = tetmesh(vertices, tetrahedra)
assert_type(mesh, SimplicialComplex)
assert_type(mesh.positions().view(), NDArray[np.float64])

scene = Scene()
created = scene.objects().create("typing-probe")
assert_type(created, Object)
assert_type(scene.objects().find(created.id()), Object)

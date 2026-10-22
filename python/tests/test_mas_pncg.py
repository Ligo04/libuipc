"""Public config selection and frame-owned force updates with MAS-PNCG."""

import numpy as np
import pytest

from uipc import Engine, Scene, World, builtin, view
from uipc.constitution import (
    ARAP,
    ElasticModuli,
    ElasticModuli2D,
    FiniteElementExternalForce,
    NeoHookeanShell,
    StableNeoHookean,
    StrainLimitingBaraffWitkinShell,
)
from uipc.geometry import ground, label_surface, label_triangle_orient, tetmesh, trimesh

from conftest import skip_cuda_on_macos, skip_cuda_on_macos_reason


@pytest.mark.cuda
@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
@pytest.mark.parametrize("update_mode", ["incremental", "frozen", "full_rebuild"])
@pytest.mark.parametrize("material", ["snh", "arap", "neo_shell", "bw_shell"])
def test_mas_pncg_config_and_animated_force_clear(tmp_path, update_mode, material):
    config = Scene.default_config()
    config["nonlinear_system"]["solver"] = "mas_pncg"
    config["nonlinear_system"]["mas_pncg"]["update_mode"] = update_mode
    config["contact"]["enable"] = False
    scene = Scene(config)
    engine = Engine("cuda", str(tmp_path / update_mode))
    world = World(engine)

    if material in ("snh", "arap"):
        initial = np.array(
            [[0.0, 0.2, 0.0], [0.1, 0.2, 0.0], [0.0, 0.3, 0.0], [0.0, 0.2, 0.1]]
        )
        mesh = tetmesh(initial, np.array([[0, 1, 2, 3]], dtype=np.int32))
        if material == "snh":
            StableNeoHookean().apply_to(
                mesh, ElasticModuli.youngs_poisson(1e4, 0.3), mass_density=1000.0
            )
        else:
            ARAP().apply_to(mesh, 1e4, mass_density=1000.0)
        vertex_mass = 1000.0 * 0.1**3 / 6 / 4
    else:
        initial = np.array([[0.0, 0.2, 0.0], [0.1, 0.2, 0.0], [0.0, 0.2, 0.1]])
        mesh = trimesh(initial, np.array([[0, 1, 2]], dtype=np.int32))
        constitution = (
            NeoHookeanShell()
            if material == "neo_shell"
            else StrainLimitingBaraffWitkinShell()
        )
        constitution.apply_to(
            mesh,
            ElasticModuli2D.youngs_poisson(1e4, 0.3),
            mass_density=200.0,
            thickness=0.001,
        )
        vertex_mass = 200.0 * 0.002 * 0.5 * 0.1**2 / 3
    label_surface(mesh)
    if material in ("snh", "arap"):
        label_triangle_orient(mesh)
    FiniteElementExternalForce().apply_to(mesh, np.array([2 * vertex_mass, 0.0, 0.0]))
    obj = scene.objects().create("tet")
    slot, _ = obj.geometries().create(mesh)
    callback_frames = []

    def animate_force(info):
        callback_frames.append(info.frame())
        geometry = info.geo_slots()[0].geometry()
        view(geometry.vertices().find("external_force/is_constrained"))[:] = (
            info.frame() == 1
        )

    scene.animator().insert(obj, animate_force)
    world.init(scene)
    assert world.is_valid()

    for frame in range(1, 9):
        world.advance()
        assert world.is_valid(), engine.frame_stats()
        world.retrieve()
        stats = engine.frame_stats()
        assert stats["completed"] and stats["converged"]
        assert stats["nonlinear_solver"] == "mas_pncg"
        assert stats["update_mode"] == update_mode
        assert stats["newton_iterations"] == stats["linear_solver_iterations"] == 0
        assert stats["max_local_residual_ratio"] <= 1
        assert callback_frames == list(range(1, frame + 1))
        force = np.array(view(slot.geometry().vertices().find("external_force")))
        np.testing.assert_allclose(
            force.reshape(-1, 3),
            np.tile([2 * vertex_mass, 0, 0], (len(initial), 1)),
            atol=1e-15,
            rtol=0,
        )
        positions = np.array(slot.geometry().positions().view()).reshape(-1, 3)
        expected = initial + [0.0002 * frame, -0.00098 * frame * (frame + 1) / 2, 0]
        np.testing.assert_allclose(positions, expected, atol=1e-9, rtol=0)


@pytest.mark.cuda
@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
def test_mas_pncg_contact_update_and_execution_modes_match_reference(tmp_path):
    trajectories = {}
    cases = [(mode, 1, 1) for mode in ("full_rebuild", "frozen", "incremental")]
    cases += [("incremental", 0, 0), ("incremental", 1, 0)]
    for mode, fused, graph in cases:
        case = f"{mode}-{fused}-{graph}"
        config = Scene.default_config()
        config["nonlinear_system"]["solver"] = "mas_pncg"
        config["nonlinear_system"]["mas_pncg"]["update_mode"] = mode
        config["nonlinear_system"]["mas_pncg"]["use_fused_kernels"] = fused
        config["nonlinear_system"]["mas_pncg"]["use_cuda_graph"] = graph
        engine = Engine("cuda", str(tmp_path / case))
        world = World(engine)
        scene = Scene(config)
        scene.contact_tabular().default_model(0.3, 1e6)
        vertices = np.array(
            [[0.0, 0.06, 0.0], [0.1, 0.06, 0.0], [0.0, 0.16, 0.0], [0.0, 0.06, 0.1]]
        )
        mesh = tetmesh(vertices, np.array([[0, 1, 2, 3]], dtype=np.int32))
        label_surface(mesh)
        label_triangle_orient(mesh)
        StableNeoHookean().apply_to(mesh, ElasticModuli.youngs_poisson(1e4, 0.3))
        view(mesh.vertices().find(builtin.velocity))[:] = [[0.05], [0.0], [0.0]]
        obj = scene.objects().create("tet-and-plane")
        slot, _ = obj.geometries().create(mesh)
        obj.geometries().create(ground(0))
        world.init(scene)
        assert world.is_valid()
        frames = []
        columns = 0
        for _ in range(45):
            world.advance()
            stats = engine.frame_stats()
            assert world.is_valid(), stats
            assert stats["completed"] and stats["converged"]
            assert stats["linear_solver_iterations"] == 0
            assert stats["fused_kernels"] == bool(fused)
            assert stats["cuda_graph_enabled"] == bool(graph)
            assert (stats["cuda_graph_replays"] > 0) == bool(graph)
            assert stats["cuda_graph_fallbacks"] == 0
            assert stats["max_local_residual_ratio"] <= 1
            columns += stats["contact_update_columns"]
            world.retrieve()
            positions = np.array(slot.geometry().positions().view()).reshape(-1, 3)
            assert np.isfinite(positions).all()
            assert (positions[:, 1] > 0).all()
            frames.append(positions)
        assert positions[:, 1].min() < 0.02
        horizontal_displacement = (positions - vertices)[:, 0].mean()
        assert 0 < horizontal_displacement < 0.75 * 0.05 * 0.01 * 45
        assert (columns > 0) == (mode == "incremental")
        trajectories[case] = np.stack(frames)
    for mode in ("frozen", "incremental"):
        np.testing.assert_allclose(
            trajectories[f"{mode}-1-1"], trajectories["full_rebuild-1-1"], atol=1e-5, rtol=0
        )
    for fused, graph in ((0, 0), (1, 0)):
        np.testing.assert_allclose(
            trajectories[f"incremental-{fused}-{graph}"],
            trajectories["incremental-1-1"], atol=1e-9, rtol=0,
        )


@pytest.mark.cuda
@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
@pytest.mark.parametrize("linear_solver", ["fused_pcg", "linear_pcg"])
def test_mas_pncg_keeps_newton_mas_selection_available(tmp_path, linear_solver):
    config = Scene.default_config()
    assert config["nonlinear_system"]["solver"] == "newton"
    config["linear_system"]["solver"] = linear_solver
    config["linear_system"]["fem_preconditioner"] = "mas"
    config["contact"]["enable"] = False
    engine = Engine("cuda", str(tmp_path / linear_solver))
    world = World(engine)
    scene = Scene(config)
    initial = np.array(
        [[0.0, 0.2, 0.0], [0.1, 0.2, 0.0], [0.0, 0.3, 0.0], [0.0, 0.2, 0.1]]
    )
    mesh = tetmesh(initial, np.array([[0, 1, 2, 3]], dtype=np.int32))
    label_surface(mesh)
    label_triangle_orient(mesh)
    StableNeoHookean().apply_to(mesh, ElasticModuli.youngs_poisson(1e4, 0.3))
    slot, _ = scene.objects().create("tet").geometries().create(mesh)
    world.init(scene)
    assert world.is_valid()
    for frame in range(1, 9):
        world.advance()
        assert world.is_valid(), engine.frame_stats()
        world.retrieve()
        stats = engine.frame_stats()
        assert stats["completed"] and stats["nonlinear_solver"] == "newton"
        assert stats["converged"]
        assert not stats["hit_newton_limit"]
        assert not stats["hit_line_search_limit"]
        assert stats["newton_iterations"] > 0
        assert stats["linear_solver_iterations"] > 0
        positions = np.array(slot.geometry().positions().view()).reshape(-1, 3)
        expected = initial + [0, -0.00098 * frame * (frame + 1) / 2, 0]
        np.testing.assert_allclose(positions, expected, atol=1e-8, rtol=0)

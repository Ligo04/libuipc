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
        assert stats["iteration_policy"] == "strict"
        assert stats["independent_residual_converged"]
        assert stats["convergence_criterion"] == "independent_residual"
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
def test_mas_pncg_mollified_edge_contacts_preserve_incremental_history(tmp_path):
    trajectories = {}
    fixed_vertices = np.array([[-0.5, 0.0, 0.0], [0.5, 0.0, 0.0], [0.0, -0.3, 0.0]])
    moving_vertices = np.array([[-0.5, 0.2, 0.01], [0.5, 0.2, -0.01], [0.0, 0.5, 0.0]])
    for fused, graph in ((0, 0), (1, 0), (1, 1)):
        config = Scene.default_config()
        config["gravity"] = [[0.0], [0.0], [0.0]]
        config["contact"]["d_hat"] = 0.3
        config["contact"]["d_hat_relative"] = 0.0
        config["nonlinear_system"]["solver"] = "mas_pncg"
        config["nonlinear_system"]["mas_pncg"]["use_fused_kernels"] = fused
        config["nonlinear_system"]["mas_pncg"]["use_cuda_graph"] = graph
        scene = Scene(config)
        scene.contact_tabular().default_model(0.0, 100.0)
        engine = Engine("cuda", str(tmp_path / f"ee-{fused}-{graph}"))
        world = World(engine)
        slots = []
        for fixed, vertices in ((True, fixed_vertices), (False, moving_vertices)):
            mesh = trimesh(vertices, np.array([[0, 1, 2]], dtype=np.int32))
            label_surface(mesh)
            NeoHookeanShell().apply_to(
                mesh,
                ElasticModuli2D.youngs_poisson(1e4, 0.3),
                mass_density=1000.0,
                thickness=0.001,
            )
            view(mesh.vertices().find(builtin.is_fixed))[:] = int(fixed)
            if not fixed:
                view(mesh.vertices().find(builtin.velocity))[:] = [[0.0], [-0.01], [0.0]]
            slot, _ = scene.objects().create(f"triangle-{fixed}").geometries().create(mesh)
            slots.append(slot)
        world.init(scene)
        assert world.is_valid()
        frames = []
        mollified = columns = history = 0
        for _ in range(4):
            world.advance()
            stats = engine.frame_stats()
            assert world.is_valid(), stats
            assert stats["completed"] and stats["converged"], stats
            assert stats["max_local_residual_ratio"] <= 1
            assert stats["contact_rebuild_fallbacks"] == 0
            assert stats["linear_solver_iterations"] == 0
            assert stats["cuda_graph_fallbacks"] == 0
            assert (stats["cuda_graph_replays"] > 0) == bool(graph)
            mollified += stats["peak_mollified_contacts"]
            columns += stats["contact_update_columns"]
            history += stats["two_d_steps"]
            world.retrieve()
            np.testing.assert_array_equal(
                np.array(slots[0].geometry().positions().view()).reshape(-1, 3),
                fixed_vertices,
            )
            positions = np.array(slots[1].geometry().positions().view()).reshape(-1, 3)
            assert np.isfinite(positions).all()
            assert positions[:, 1].min() > 0
            frames.append(positions)
        assert mollified > 0
        assert columns > 0
        assert history > 0
        trajectories[(fused, graph)] = np.stack(frames)
    for mode in ((1, 0), (1, 1)):
        np.testing.assert_allclose(trajectories[mode], trajectories[(0, 0)], atol=1e-9, rtol=0)


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


def _mas_pncg_paper_config(fused, graph):
    config = Scene.default_config()
    config["nonlinear_system"]["solver"] = "mas_pncg"
    pncg = config["nonlinear_system"]["mas_pncg"]
    pncg["iteration_policy"] = "paper"
    pncg["use_fused_kernels"] = fused
    pncg["use_cuda_graph"] = graph
    return config


def _assert_mas_pncg_paper_stats(stats, fused, graph):
    assert stats["nonlinear_solver"] == "mas_pncg"
    assert stats["iteration_policy"] == "paper"
    assert stats["newton_iterations"] == stats["linear_solver_iterations"] == 0
    assert stats["fused_kernels"] == bool(fused)
    assert stats["cuda_graph_enabled"] == bool(graph)
    assert stats["cuda_graph_fallbacks"] == 0
    assert isinstance(stats["independent_residual_converged"], bool)
    assert isinstance(stats["convergence_checked_on_rebuild"], bool)
    assert stats["energy_increase_steps"] >= 0
    assert stats["orthogonality_restarts"] >= 0
    assert stats["numerical_retries"] >= 0
    for name in (
        "gradient_initial",
        "gradient_final",
        "max_local_residual_ratio",
        "preconditioned_gradient_norm",
        "preconditioned_gradient_tolerance",
    ):
        assert np.isfinite(stats[name]), (name, stats)
    assert stats["preconditioned_gradient_tolerance"] > 0


@pytest.mark.cuda
@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
@pytest.mark.parametrize("fixed", [False, True])
@pytest.mark.parametrize("fused,graph", [(0, 0), (1, 0), (1, 1)])
def test_mas_pncg_paper_preserves_resting_and_fixed_states(tmp_path, fixed, fused, graph):
    config = _mas_pncg_paper_config(fused, graph)
    config["contact"]["enable"] = False
    if not fixed:
        config["gravity"] = [[0.0], [0.0], [0.0]]
    scene = Scene(config)
    if not fixed:
        np.testing.assert_array_equal(
            np.array(scene.config().find("gravity").view()).reshape(3), np.zeros(3)
        )
    engine = Engine("cuda", str(tmp_path / f"resting-{fixed}-{fused}-{graph}"))
    world = World(engine)
    initial = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    )
    mesh = tetmesh(initial, np.array([[0, 1, 2, 3]], dtype=np.int32))
    label_surface(mesh)
    label_triangle_orient(mesh)
    StableNeoHookean().apply_to(mesh, ElasticModuli.youngs_poisson(1e4, 0.3))
    view(mesh.vertices().find(builtin.is_fixed))[:] = int(fixed)
    slot, _ = scene.objects().create("resting-tet").geometries().create(mesh)
    world.init(scene)
    assert world.is_valid()
    for _ in range(3):
        world.advance()
        stats = engine.frame_stats()
        assert world.is_valid(), stats
        _assert_mas_pncg_paper_stats(stats, fused, graph)
        assert stats["completed"] and stats["converged"], stats
        assert stats["independent_residual_converged"]
        if fixed:
            assert stats["convergence_criterion"] == "zero_gradient"
            assert stats["accepted_steps"] == 0
            assert stats["gradient_final"] == 0
        else:
            criterion = stats["convergence_criterion"]
            assert criterion in ("zero_gradient", "preconditioned_gradient")
            if criterion == "zero_gradient":
                assert stats["gradient_final"] == 0
            else:
                assert stats["accepted_steps"] > 0
                assert stats["convergence_checked_on_rebuild"]
                assert stats["preconditioned_gradient_norm"] <= (
                    stats["preconditioned_gradient_tolerance"]
                )
        world.retrieve()
        positions = np.array(slot.geometry().positions().view()).reshape(-1, 3)
        if fixed:
            np.testing.assert_array_equal(positions, initial)
        else:
            np.testing.assert_allclose(positions, initial, atol=1e-12, rtol=0)


def _paper_tet_plane(scene):
    initial = np.array(
        [[0.0, 0.006, 0.0], [0.1, 0.006, 0.0],
         [0.0, 0.106, 0.0], [0.0, 0.006, 0.1]]
    )
    scene.contact_tabular().default_model(0.0, 1e4)
    mesh = tetmesh(initial, np.array([[0, 1, 2, 3]], dtype=np.int32))
    label_surface(mesh)
    label_triangle_orient(mesh)
    StableNeoHookean().apply_to(
        mesh, ElasticModuli.youngs_poisson(1e4, 0.3), mass_density=1000.0
    )
    view(mesh.vertices().find(builtin.velocity))[:] = [[0.05], [-0.1], [0.0]]
    obj = scene.objects().create("tet-and-plane")
    slot, _ = obj.geometries().create(mesh)
    obj.geometries().create(ground(0))
    return slot, initial


@pytest.mark.cuda
@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
def test_mas_pncg_paper_rebuild_convergence_matches_execution_modes(tmp_path):
    trajectories = {}
    for fused, graph in ((0, 0), (1, 0), (1, 1)):
        config = _mas_pncg_paper_config(fused, graph)
        config["contact"]["d_hat"] = 0.01
        config["contact"]["d_hat_relative"] = 0.0
        pncg = config["nonlinear_system"]["mas_pncg"]
        pncg["gradient_atol"] = 1e-16
        pncg["gradient_rtol"] = 1e-16
        pncg["max_base_age"] = 4
        scene = Scene(config)
        slot, initial = _paper_tet_plane(scene)
        engine = Engine("cuda", str(tmp_path / f"rebuild-{fused}-{graph}"))
        world = World(engine)
        world.init(scene)
        assert world.is_valid()
        frames = []
        converged_before_residual = False
        for _ in range(4):
            world.advance()
            stats = engine.frame_stats()
            assert world.is_valid(), stats
            _assert_mas_pncg_paper_stats(stats, fused, graph)
            assert stats["completed"] and stats["converged"], stats
            assert stats["convergence_criterion"] == "preconditioned_gradient", stats
            assert stats["convergence_checked_on_rebuild"]
            assert stats["accepted_steps"] > 0
            assert stats["preconditioned_gradient_tolerance"] == pytest.approx(1e-5)
            assert 0 < stats["preconditioned_gradient_norm"] <= 1e-5
            assert not stats["hit_newton_limit"]
            assert not stats["hit_line_search_limit"]
            assert (stats["cuda_graph_replays"] > 0) == bool(graph)
            converged_before_residual |= not stats["independent_residual_converged"]
            world.retrieve()
            positions = np.array(slot.geometry().positions().view()).reshape(-1, 3)
            assert np.isfinite(positions).all()
            assert (positions[:, 1] > 0).all()
            frames.append(positions)
        assert converged_before_residual
        assert not np.allclose(frames[-1], initial, atol=1e-6, rtol=0)
        trajectories[(fused, graph)] = np.stack(frames)
    for mode in ((1, 0), (1, 1)):
        np.testing.assert_allclose(
            trajectories[mode], trajectories[(0, 0)], atol=1e-7, rtol=0
        )


@pytest.mark.cuda
@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
@pytest.mark.parametrize("fused,graph", [(0, 0), (1, 0), (1, 1)])
def test_mas_pncg_paper_budget_exhaustion_is_not_convergence(tmp_path, fused, graph):
    config = _mas_pncg_paper_config(fused, graph)
    config["contact"]["d_hat"] = 0.01
    config["contact"]["d_hat_relative"] = 0.0
    config["nonlinear_system"]["mas_pncg"]["max_iter"] = 1
    config["nonlinear_system"]["mas_pncg"]["preconditioned_gradient_tol"] = 1e-30
    scene = Scene(config)
    _paper_tet_plane(scene)
    engine = Engine("cuda", str(tmp_path / f"budget-{fused}-{graph}"))
    world = World(engine)
    world.init(scene)
    assert world.is_valid()
    world.advance()
    stats = engine.frame_stats()
    _assert_mas_pncg_paper_stats(stats, fused, graph)
    assert not world.is_valid()
    assert not stats["completed"] and not stats["converged"]
    assert stats["convergence_criterion"] == "not_converged"
    assert stats["hit_newton_limit"]
    assert stats["accepted_steps"] == stats["nonlinear_iterations"] == 1
    assert stats["preconditioned_gradient_norm"] > 1e-30

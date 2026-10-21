from __future__ import annotations

import json

import pytest

from uipc import Exception as UIPCException
from uipc import Scene
from uipc.cli.config_schema import main as config_schema_main


def test_scene_config_schema_matches_public_defaults() -> None:
    schema = Scene.config_schema()
    entries = schema["entries"]

    assert schema["schemaVersion"] == 1
    assert schema["strictUnknownKeys"] is True
    assert len(entries) == 59
    assert entries["nonlinear_system/solver"]["default"] == "newton"
    assert entries["nonlinear_system/solver"]["enum"] == ["newton", "mas_pncg"]
    assert entries["nonlinear_system/mas_pncg/ccd_mode"]["const"] == "global"
    assert entries["dt"]["default"] == pytest.approx(0.01)
    assert entries["dt"]["exclusiveMinimum"] == 0.0
    assert entries["gravity"]["componentCount"] == 3
    assert entries["newton/use_adaptive_tol"]["status"] == "reserved"
    assert entries["sanity_check/mode"]["enum"] == ["normal", "quiet"]
    assert entries["contact/al-ipc/mu_scale_mode"]["default"] == "per_vertex"
    assert entries["contact/al-ipc/mu_scale_mode"]["enum"] == [
        "diag_norm",
        "per_vertex",
    ]
    assert entries["contact/al-ipc/mu_scale_diag_norm"]["default"] == pytest.approx(0.1)

    collision = entries["collision_detection/method"]
    conditional = collision["conditionalValues"]
    assert conditional["backend"] == "cuda"
    assert conditional["buildOption"] == "UIPC_WITH_CUDA_LEGACY_COLLISION"
    assert collision["enum"][0] == "info_stackless_bvh"
    for legacy_method in conditional["values"]:
        assert (legacy_method in collision["enum"]) is conditional["enabled"]

    for entry in entries.values():
        assert {
            "default",
            "type",
            "storageType",
            "description",
            "consumers",
            "lifecycle",
            "status",
        } <= entry.keys()


@pytest.mark.parametrize(
    "mutate",
    [
        lambda config: config.__setitem__("dt", 0.0),
        lambda config: config["integrator"].__setitem__("type", "rk4"),
        lambda config: config["newton"].__setitem__("use_adaptive_tol", 1),
        lambda config: config["contact"]["al-ipc"].__setitem__(
            "mu_scale_mode", "unknown"
        ),
    ],
)
def test_scene_config_rejects_invalid_values(mutate) -> None:
    config = Scene.default_config()
    mutate(config)
    with pytest.raises(UIPCException):
        Scene(config)


def test_config_schema_cli_selects_one_key(capsys) -> None:
    assert config_schema_main(["dt"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["default"] == pytest.approx(0.01)
    assert payload["unit"] == "s"


def test_mas_pncg_selection_preserves_linear_solver_choice() -> None:
    config = Scene.default_config()
    config["nonlinear_system"]["solver"] = "mas_pncg"
    scene = Scene(config)
    assert scene.config().find("nonlinear_system/solver") is not None
    scene.validate_config()
    assert config["linear_system"]["solver"] == "fused_pcg"


@pytest.mark.parametrize(
    "mutate",
    [
        lambda c: c["integrator"].__setitem__("type", "bdf2"),
        lambda c: c["contact"].__setitem__("constitution", "al-ipc"),
        lambda c: c["diff_sim"].__setitem__("enable", 1),
        lambda c: c["nonlinear_system"]["mas_pncg"].__setitem__("top_k", 9),
        lambda c: c["nonlinear_system"]["mas_pncg"].__setitem__("ccd_mode", "localized"),
    ],
)
def test_mas_pncg_rejects_unsupported_config(mutate) -> None:
    config = Scene.default_config()
    config["nonlinear_system"]["solver"] = "mas_pncg"
    mutate(config)
    with pytest.raises(UIPCException):
        Scene(config)

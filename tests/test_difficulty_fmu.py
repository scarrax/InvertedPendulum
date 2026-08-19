import math
import os

import pytest

from pendulum_game_controlled import apply_difficulty_physics, reset_round, DIFFICULTY_LEVELS

FMU_PATH = os.path.abspath("InvertedPendulumMB.fmu")


def _instantiate_fmu():
    from fmpy import read_model_description, extract
    from fmpy.fmi2 import FMU2Slave

    unzipdir = extract(FMU_PATH)
    desc = read_model_description(unzipdir)
    fmu = FMU2Slave(
        guid=desc.guid,
        unzipDirectory=unzipdir,
        modelIdentifier=desc.coSimulation.modelIdentifier,
    )
    fmu.instantiate()
    fmu.setupExperiment(startTime=0.0)

    def ref(name):
        for var in desc.modelVariables:
            if var.name == name:
                return var.valueReference
        raise Exception(f"'{name}' not found in FMU")

    return fmu, ref


@pytest.mark.skipif(
    not os.path.exists(FMU_PATH),
    reason="InvertedPendulumMB.fmu not present (gitignored build artifact, copy manually into worktrees that run the game)",
)
@pytest.mark.parametrize("difficulty", ["Leicht", "Standard", "Schwer"])
def test_apply_difficulty_physics_sets_expected_values(difficulty):
    fmu, ref = _instantiate_fmu()
    value_refs = {
        "m_cart": ref("m_cart"),
        "m_pend": ref("m_pend"),
        "d_cart": ref("d_cart"),
        "d_pend": ref("d_pend"),
    }

    apply_difficulty_physics(fmu, value_refs, difficulty)
    fmu.enterInitializationMode()
    fmu.exitInitializationMode()

    level = DIFFICULTY_LEVELS[difficulty]
    assert math.isclose(fmu.getReal([value_refs["m_cart"]])[0], level["m_cart"])
    assert math.isclose(fmu.getReal([value_refs["m_pend"]])[0], level["m_pend"])
    assert math.isclose(fmu.getReal([value_refs["d_cart"]])[0], level["d_cart"])
    assert math.isclose(fmu.getReal([value_refs["d_pend"]])[0], level["d_pend"])

    fmu.terminate()
    fmu.freeInstance()


@pytest.mark.skipif(
    not os.path.exists(FMU_PATH),
    reason="InvertedPendulumMB.fmu not present (gitignored build artifact, copy manually into worktrees that run the game)",
)
def test_reset_round_applies_difficulty_and_returns_fresh_state():
    fmu, ref = _instantiate_fmu()
    value_refs = {
        "m_cart": ref("m_cart"),
        "m_pend": ref("m_pend"),
        "d_cart": ref("d_cart"),
        "d_pend": ref("d_pend"),
    }
    s_ref, v_ref, phi_ref, vphi_ref = ref("s"), ref("v"), ref("phi"), ref("vphi")

    # Einmal durch die Initialisierung laufen, wie beim echten Spielstart.
    fmu.enterInitializationMode()
    fmu.exitInitializationMode()

    s, v, phi, vphi = reset_round(fmu, value_refs, "Schwer", s_ref, v_ref, phi_ref, vphi_ref)

    level = DIFFICULTY_LEVELS["Schwer"]
    assert math.isclose(fmu.getReal([value_refs["m_cart"]])[0], level["m_cart"])
    assert math.isclose(fmu.getReal([value_refs["m_pend"]])[0], level["m_pend"])
    assert math.isclose(fmu.getReal([value_refs["d_cart"]])[0], level["d_cart"])
    assert math.isclose(fmu.getReal([value_refs["d_pend"]])[0], level["d_pend"])
    # Ein frischer Reset startet beim Modell-Default-Anfangswinkel (~67.5°
    # aus der hängenden Ruhelage), unabhängig von den Difficulty-Physik-Werten.
    assert math.isclose(math.degrees(phi), 67.5, rel_tol=1e-3)
    assert math.isclose(vphi, 0.0, abs_tol=1e-6)

    fmu.terminate()
    fmu.freeInstance()

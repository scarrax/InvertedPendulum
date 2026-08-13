import math
import os

import pytest

FMU_PATH = os.path.abspath("InvertedPendulumMB.fmu")


@pytest.mark.skipif(
    not os.path.exists(FMU_PATH),
    reason="InvertedPendulumMB.fmu not present (gitignored build artifact, copy manually into worktrees that run the game)",
)
def test_substepped_free_swing_stays_energy_bounded():
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

    fmu.enterInitializationMode()
    fmu.exitInitializationMode()
    tau_ref, phi_ref = ref("tau"), ref("phi")

    dt = 0.02
    SUBSTEPS = 10
    substep_dt = dt / SUBSTEPS
    t = 0.0
    max_phi_deg = -1e9
    min_phi_deg = 1e9
    while t < 40.0:
        fmu.setReal([tau_ref], [0.0])
        for _ in range(SUBSTEPS):
            t += substep_dt
            fmu.doStep(currentCommunicationPoint=t, communicationStepSize=substep_dt)
        phi_deg = math.degrees(fmu.getReal([phi_ref])[0])
        max_phi_deg = max(max_phi_deg, phi_deg)
        min_phi_deg = min(min_phi_deg, phi_deg)

    fmu.terminate()
    fmu.freeInstance()

    # Energy-conservation bound: starting at phi0~67.5 deg from hanging with
    # zero velocity and tau=0 for the whole run, the pendulum can never swing
    # past its own starting height (dissipation only removes energy). 90 deg
    # gives comfortable margin above the theoretical ~67.5 deg limit while
    # still catching a reintroduced SUBSTEPS=1 regression, which blows past
    # 180 deg within a few seconds (see pendulum_game_controlled.py's
    # SUBSTEPS comment and AP1_Validierung.md §6.7's "Nachtrag" paragraph).
    assert max_phi_deg < 90.0
    assert min_phi_deg > -90.0

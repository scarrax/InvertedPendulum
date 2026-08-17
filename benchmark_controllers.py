"""Headless benchmark comparing the three AP3 controllers against the real
InvertedPendulumMB.fmu on stability, reaction time, and robustness.

See docs/superpowers/specs/2026-08-17-ap3-controller-comparison-design.md
for the full methodology.
"""

import math
import os
import shutil

from fmpy import extract, read_model_description
from fmpy.fmi2 import FMU2Slave

# Success criterion shared across all scenarios: a controller "holds" the
# upright position from the moment |theta| stays below TOLERANCE_DEG for at
# least HOLD_DURATION seconds. TOLERANCE_DEG matches the game's own
# tight_bonus_zone (pendulum_game_controlled.py's compute_score_increment).
TOLERANCE_DEG = 5.0
HOLD_DURATION = 1.0


def held_from(t, theta, tolerance_rad, hold_duration):
    """Return the earliest time in `t` from which |theta| stays below
    `tolerance_rad` for at least `hold_duration` seconds, or None if that
    never happens within the given data."""
    entry_time = None
    for ti, th in zip(t, theta):
        if abs(th) < tolerance_rad:
            if entry_time is None:
                entry_time = ti
            elif ti - entry_time >= hold_duration:
                return entry_time
        else:
            entry_time = None
    return None


def find_capture_envelope(results_by_theta0):
    """results_by_theta0: dict {theta0_deg: bool success}, assumed to come
    from a monotonically increasing sweep. Return the largest theta0_deg
    for which every smaller tested theta0_deg also succeeded (i.e. the
    value just before the first failure), or None if the smallest tested
    theta0_deg already failed or the input is empty."""
    best = None
    for theta0_deg in sorted(results_by_theta0):
        if results_by_theta0[theta0_deg]:
            best = theta0_deg
        else:
            break
    return best


def simulate_run(fmu_path, controller, theta0_deg=None, vphi0=0.0, duration=20.0, on_frame=None):
    # Sub-stepped FMU co-simulation loop, same pattern as run_game() in
    # pendulum_game_controlled.py: explicit-Euler at the full 0.02s step
    # numerically injects energy into the lightly-damped pendulum, so tau
    # is held constant across SUBSTEPS smaller inner steps.
    dt = 0.02
    SUBSTEPS = 10
    MAX_TAU = 10.0
    substep_dt = dt / SUBSTEPS

    unzipdir = extract(fmu_path)
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

    if theta0_deg is not None:
        fmu.setReal([ref("phi0")], [math.pi + math.radians(theta0_deg)])
        fmu.setReal([ref("vphi0")], [vphi0])

    fmu.enterInitializationMode()
    fmu.exitInitializationMode()

    tau_ref = ref("tau")
    s_ref = ref("s")
    v_ref = ref("v")
    phi_ref = ref("phi")
    vphi_ref = ref("vphi")

    def wrapped_theta(phi):
        return (phi % (2 * math.pi)) - math.pi

    t = 0.0
    t_history = [0.0]
    theta_history = [wrapped_theta(fmu.getReal([phi_ref])[0])]

    try:
        while t < duration:
            phi = fmu.getReal([phi_ref])[0]
            vphi = fmu.getReal([vphi_ref])[0]
            s = fmu.getReal([s_ref])[0]
            v = fmu.getReal([v_ref])[0]
            theta = wrapped_theta(phi)

            tau = controller.compute(phi, vphi, s, v)
            if on_frame is not None:
                tau += on_frame(t, theta, vphi, s, v)
            tau = max(-MAX_TAU, min(MAX_TAU, tau))

            fmu.setReal([tau_ref], [tau])
            for _ in range(SUBSTEPS):
                t += substep_dt
                fmu.doStep(currentCommunicationPoint=t, communicationStepSize=substep_dt)

            t_history.append(t)
            theta_history.append(wrapped_theta(fmu.getReal([phi_ref])[0]))
    finally:
        fmu.terminate()
        fmu.freeInstance()
        shutil.rmtree(unzipdir)

    return t_history, theta_history

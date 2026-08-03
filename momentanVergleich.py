import numpy as np
import pandas as pd
from fmpy import read_model_description, extract
from fmpy.fmi2 import FMU2Slave
import shutil
import os


def get_derivatives(fmu_path, s0, v0, phi0, vphi0, tau, a_name, alpha_name):
    unzipdir = extract(os.path.abspath(fmu_path))
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

    fmu.setReal(
        [ref("s0"), ref("v0"), ref("phi0"), ref("vphi0")],
        [s0, v0, phi0, vphi0],
    )
    fmu.enterInitializationMode()
    fmu.exitInitializationMode()

    fmu.setReal([ref("tau")], [tau])

    a, alpha = fmu.getReal([ref(a_name), ref(alpha_name)])

    fmu.terminate()
    fmu.freeInstance()
    shutil.rmtree(unzipdir)

    return a, alpha


def compare_states(fmu_flat, fmu_mb, states):
    rows = []
    for s0, v0, phi0, vphi0, tau in states:
        a_flat, alpha_flat = get_derivatives(fmu_flat, s0, v0, phi0, vphi0, tau, "a", "alpha")
        a_mb, alpha_mb = get_derivatives(fmu_mb, s0, v0, phi0, vphi0, tau, "prismatic.a", "revolute.a")
        rows.append({
            "s0": s0, "v0": v0, "phi0": phi0, "vphi0": vphi0, "tau": tau,
            "a_flat": a_flat, "a_mb": a_mb, "a_diff": abs(a_flat - a_mb),
            "alpha_flat": alpha_flat, "alpha_mb": alpha_mb,
            "alpha_diff": abs(alpha_flat - alpha_mb),
        })
    return pd.DataFrame(rows)


if __name__ == "__main__":
    states = [
        (0.0, 0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0, 2.0),
        (0.0, 0.0, 0.0, 0.0, -3.0),
        (0.0, 1.0, 0.0, 0.0, 0.0),
        (0.0, -2.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, np.pi / 2, 0.0, 0.0),
        (0.0, 0.0, np.pi, 0.0, 0.0),
        (0.0, 1.0, 1.178097, 0.5, 2.0),
        (1.0, -0.5, np.pi / 2, -1.0, -3.0),
    ]

    result = compare_states("InvertedPendulum.fmu", "InvertedPendulumMB.fmu", states)
    print(result.to_string())
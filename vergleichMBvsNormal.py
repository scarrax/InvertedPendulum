import numpy as np
import pandas as pd
from fmpy import read_model_description, extract
from fmpy.fmi2 import FMU2Slave
import shutil
import os


def simulate(fmu_path, dt=0.02, duration=20.0):
    unzipdir = extract(os.path.abspath(fmu_path))
    desc = read_model_description(unzipdir)
    fmu = FMU2Slave(
        guid=desc.guid,
        unzipDirectory=unzipdir,
        modelIdentifier=desc.coSimulation.modelIdentifier,
    )
    fmu.instantiate()
    fmu.setupExperiment(startTime=0.0)
    fmu.enterInitializationMode()
    fmu.exitInitializationMode()

    def ref(name):
        for var in desc.modelVariables:
            if var.name == name:
                return var.valueReference
        raise Exception(f"'{name}' not found in FMU")

    tau_ref = ref("tau")
    s_ref, v_ref = ref("s"), ref("v")
    phi_ref, vphi_ref = ref("phi"), ref("vphi")

    rows = []
    time = 0.0
    while time < duration:
        # Reproduzierbares Testsignal, kein Regler
        tau = 5.0 * np.sin(0.5 * time)

        fmu.setReal([tau_ref], [tau])
        time += dt
        fmu.doStep(currentCommunicationPoint=time, communicationStepSize=dt)

        s, v = fmu.getReal([s_ref, v_ref])
        phi, vphi = fmu.getReal([phi_ref, vphi_ref])

        rows.append({"time": time, "s": s, "v": v, "phi": phi, "vphi": vphi, "tau": tau})

    fmu.terminate()
    fmu.freeInstance()
    shutil.rmtree(unzipdir)

    return pd.DataFrame(rows)


def compare(df_ref, df_new, columns=("s", "v", "phi", "vphi")):
    results = {}
    for col in columns:
        rmse = np.sqrt(np.mean((df_ref[col] - df_new[col]) ** 2))
        max_err = np.max(np.abs(df_ref[col] - df_new[col]))
        results[col] = {"rmse": rmse, "max_abs_error": max_err}
    return pd.DataFrame(results).T


if __name__ == "__main__":
    print("Starte Simulation InvertedPendulum.fmu ...")
    df_flat = simulate("InvertedPendulum.fmu", duration=20.0)
    print("Fertig, Zeilen:", len(df_flat))

    print("Starte Simulation InvertedPendulumMB.fmu ...")
    df_mb = simulate("InvertedPendulumMB.fmu", duration=20.0)
    print("Fertig, Zeilen:", len(df_mb))

    df_flat.to_csv("sim_flat.csv", index=False)
    df_mb.to_csv("sim_mb.csv", index=False)
    print("CSV-Dateien geschrieben.")

    result = compare(df_flat, df_mb)
    print(result)
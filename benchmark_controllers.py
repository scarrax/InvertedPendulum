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

from pendulum_game_controlled import LQRController, SimpleController, SwingUpController

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


CONTROLLER_FACTORIES = {
    "PD": SimpleController,
    "LQR": LQRController,
    "SwingUp": SwingUpController,
}


class KickInjector:
    """on_frame callback for simulate_run: waits until the controller has
    held the upright position, then injects a fixed-magnitude tau pulse for
    `kick_steps` frames. Records when the kick started (`kick_time`, None
    if the controller never settled)."""

    def __init__(self, tolerance_rad, hold_duration, kick_tau, kick_steps):
        self.tolerance_rad = tolerance_rad
        self.hold_duration = hold_duration
        self.kick_tau = kick_tau
        self.kick_steps = kick_steps
        self.kick_time = None
        self._t_history = []
        self._theta_history = []
        self._kicked = False
        self._kick_frames_remaining = 0

    def offset_for(self, t, theta, vphi=None, s=None, v=None):
        self._t_history.append(t)
        self._theta_history.append(theta)

        if self._kick_frames_remaining > 0:
            self._kick_frames_remaining -= 1
            return self.kick_tau

        if not self._kicked:
            if held_from(self._t_history, self._theta_history, self.tolerance_rad, self.hold_duration) is not None:
                self._kicked = True
                self.kick_time = t
                self._kick_frames_remaining = self.kick_steps - 1
                return self.kick_tau

        return 0.0


def envelope_sweep(fmu_path, controller_factory, theta0_values_deg=None,
                    tolerance_deg=TOLERANCE_DEG, hold_duration=HOLD_DURATION,
                    duration=20.0, simulate_fn=simulate_run):
    if theta0_values_deg is None:
        theta0_values_deg = list(range(2, 92, 2))
    tolerance_rad = math.radians(tolerance_deg)

    results_by_theta0 = {}
    consecutive_failures = 0
    for theta0_deg in theta0_values_deg:
        controller = controller_factory()
        t, theta = simulate_fn(fmu_path, controller, theta0_deg=theta0_deg, duration=duration)
        success = held_from(t, theta, tolerance_rad, hold_duration) is not None
        results_by_theta0[theta0_deg] = success
        consecutive_failures = 0 if success else consecutive_failures + 1
        if consecutive_failures >= 2:
            break

    return {
        "results_by_theta0": results_by_theta0,
        "envelope_deg": find_capture_envelope(results_by_theta0),
    }


def reaction_time(fmu_path, controller_factory, theta0_values_deg=(2.0, 10.0),
                   tolerance_deg=TOLERANCE_DEG, hold_duration=HOLD_DURATION,
                   duration=20.0, simulate_fn=simulate_run):
    tolerance_rad = math.radians(tolerance_deg)
    settling_times = {}
    trajectories = {}
    for theta0_deg in theta0_values_deg:
        controller = controller_factory()
        t, theta = simulate_fn(fmu_path, controller, theta0_deg=theta0_deg, duration=duration)
        settling_times[theta0_deg] = held_from(t, theta, tolerance_rad, hold_duration)
        trajectories[theta0_deg] = (t, theta)
    return {"settling_times": settling_times, "trajectories": trajectories}


def robustness(fmu_path, controller_factory, theta0_deg=2.0,
                tolerance_deg=TOLERANCE_DEG, hold_duration=HOLD_DURATION,
                kick_tau=8.0, kick_steps=5, duration=20.0, simulate_fn=simulate_run):
    tolerance_rad = math.radians(tolerance_deg)
    injector = KickInjector(tolerance_rad, hold_duration, kick_tau, kick_steps)
    controller = controller_factory()
    t, theta = simulate_fn(
        fmu_path, controller, theta0_deg=theta0_deg, duration=duration, on_frame=injector.offset_for
    )

    if injector.kick_time is None:
        return {"kicked": False, "recovery_time": None, "kick_time": None, "trajectory": (t, theta)}

    post_kick_t = [ti for ti in t if ti >= injector.kick_time]
    post_kick_theta = [th for ti, th in zip(t, theta) if ti >= injector.kick_time]
    recovered_at = held_from(post_kick_t, post_kick_theta, tolerance_rad, hold_duration)
    recovery_time = None if recovered_at is None else recovered_at - injector.kick_time

    return {
        "kicked": True,
        "recovery_time": recovery_time,
        "kick_time": injector.kick_time,
        "trajectory": (t, theta),
    }


def swingup_capture(fmu_path, controller_factory, tolerance_deg=TOLERANCE_DEG,
                     hold_duration=HOLD_DURATION, duration=20.0, simulate_fn=simulate_run):
    tolerance_rad = math.radians(tolerance_deg)
    controller = controller_factory()
    t, theta = simulate_fn(fmu_path, controller, theta0_deg=None, duration=duration)
    capture_time = held_from(t, theta, tolerance_rad, hold_duration)
    return {"capture_time": capture_time, "trajectory": (t, theta)}


import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def run_all_scenarios(fmu_path):
    envelope = {
        name: envelope_sweep(fmu_path, factory)
        for name, factory in CONTROLLER_FACTORIES.items()
        if name != "SwingUp"
    }
    reaction = {
        name: reaction_time(fmu_path, factory)
        for name, factory in CONTROLLER_FACTORIES.items()
    }
    robust = {
        name: robustness(fmu_path, factory)
        for name, factory in CONTROLLER_FACTORIES.items()
    }
    swingup = swingup_capture(fmu_path, CONTROLLER_FACTORIES["SwingUp"])
    return {
        "envelope": envelope,
        "reaction_time": reaction,
        "robustness": robust,
        "swingup_capture": swingup,
    }


def generate_report(results):
    lines = ["# AP3 Reglervergleich", ""]

    lines.append("## 1. Kontext")
    lines.append("")
    lines.append(
        "Vergleich der drei AP3-Regler (PD/SimpleController, LQR, SwingUp) "
        "gegen die reale InvertedPendulumMB.fmu, gefordert durch die "
        "Projektanweisung (AP3: 'Vergleich der Regler hinsichtlich "
        "Stabilitaet, Reaktionszeit und Robustheit gegenueber "
        "Stoerungen'). Details zur Methodik in "
        "docs/superpowers/specs/2026-08-17-ap3-controller-comparison-design.md."
    )
    lines.append("")

    lines.append("## 2. Methodik")
    lines.append("")
    lines.append(
        f"Erfolgskriterium: |theta| < {TOLERANCE_DEG:.0f}° fuer mindestens "
        f"{HOLD_DURATION:.1f}s ununterbrochen (theta = Abweichung von der "
        "aufrechten Lage phi=pi). Stabilitaet wird als groesste "
        "erfolgreiche Anfangsauslenkung im 2°-Sweep gemessen (PD/LQR), "
        "Reaktionszeit als Einschwingzeit bei festen "
        "Baseline-Auslenkungen (2°, 10°), Robustheit als Erholungszeit "
        "nach einem Kraft-Puls auf tau (KICK_TAU=8.0 fuer KICK_STEPS=5 "
        "Frames) waehrend des eingeschwungenen Zustands, und die "
        "Swing-up-Faehigkeit als Einschwingzeit ab der realen "
        "Spiel-Anfangsbedingung (phi0=0.75*pi/2)."
    )
    lines.append("")

    lines.append("## 3. Stabilitaet (Einzugsbereich)")
    lines.append("")
    lines.append("| Regler | Einzugsbereich |")
    lines.append("|---|---|")
    for name, data in results["envelope"].items():
        envelope_deg = data["envelope_deg"]
        value = "kein Erfolg im gesweepten Bereich" if envelope_deg is None else f"{envelope_deg:.0f}°"
        lines.append(f"| {name} | {value} |")
    lines.append("| SwingUp | siehe Swing-up-Ergebnis unten |")
    lines.append("")

    lines.append("## 4. Reaktionszeit")
    lines.append("")
    lines.append("| Regler | 2° | 10° |")
    lines.append("|---|---|---|")
    for name, data in results["reaction_time"].items():
        row = [name]
        for theta0 in (2.0, 10.0):
            t = data["settling_times"].get(theta0)
            row.append("kein Einschwingen" if t is None else f"{t:.2f}s")
        lines.append("| " + " | ".join(row) + " |")
    lines.append("")

    lines.append("## 5. Robustheit (Kraft-Puls)")
    lines.append("")
    lines.append("| Regler | Erholungszeit |")
    lines.append("|---|---|")
    for name, data in results["robustness"].items():
        if not data["kicked"]:
            value = "Regler hat vor dem Puls nicht eingeschwungen"
        elif data["recovery_time"] is None:
            value = "keine Erholung"
        else:
            value = f"{data['recovery_time']:.2f}s"
        lines.append(f"| {name} | {value} |")
    lines.append("")

    lines.append("## 6. Swing-up ab realer Anfangsbedingung")
    lines.append("")
    capture_time = results["swingup_capture"]["capture_time"]
    value = "kein Capture" if capture_time is None else f"{capture_time:.2f}s"
    lines.append(f"SwingUp: {value}")
    lines.append("PD/LQR: N/A (strukturell nicht loesbar, siehe Design-Dokument §3.4)")
    lines.append("")

    lines.append("## 7. Diskussion")
    lines.append("")
    lines.append(
        "Die drei Regler unterscheiden sich strukturell in ihrem "
        "Einzugsbereich (Tabelle oben): PD reagiert nur lokal um phi=pi, "
        "LQR nutzt vollen Zustand (s, v, theta, theta_dot) und deckt "
        "typischerweise einen groesseren Bereich ab, aber beide sind auf "
        "eine Linearisierung um die aufrechte Lage angewiesen und koennen "
        "die reale Spiel-Anfangsbedingung (~112.5° von der aufrechten "
        "Lage) strukturell nicht erreichen (Abschnitt 6) - genau der in "
        "der Projektanweisung genannte Vergleichspunkt fuer den "
        "energiebasierten SwingUp-Regler (siehe AP1_Validierung.md §6 "
        "fuer die zugrundeliegende Physik). Bei der Robustheit "
        "(Abschnitt 5) zeigt die Erholungszeit nach dem Kraft-Puls, "
        "welcher Regler eine Stoerung am schnellsten wieder ausregelt; "
        "ein Regler, der vor dem Puls gar nicht erst eingeschwungen war, "
        "wird als 'nicht eingeschwungen' statt mit einer Erholungszeit "
        "gefuehrt und ist entsprechend gesondert zu lesen."
    )
    lines.append("")

    return "\n".join(lines)


def generate_plots(results, output_dir):
    os.makedirs(output_dir, exist_ok=True)

    names = list(results["envelope"].keys())
    values = [(results["envelope"][n]["envelope_deg"] or 0) for n in names]
    plt.figure()
    plt.bar(names, values)
    plt.ylabel("Einzugsbereich (Grad)")
    plt.title("Stabilitaet: Einzugsbereich je Regler")
    plt.savefig(os.path.join(output_dir, "envelope_sweep.png"))
    plt.close()

    for theta0 in (2.0, 10.0):
        plt.figure()
        for name, data in results["reaction_time"].items():
            t, theta = data["trajectories"][theta0]
            plt.plot(t, [math.degrees(th) for th in theta], label=name)
        plt.axhline(5.0, color="gray", linestyle="--")
        plt.axhline(-5.0, color="gray", linestyle="--")
        plt.xlabel("t (s)")
        plt.ylabel("theta (deg)")
        plt.title(f"Reaktionszeit ab {theta0:.0f}°")
        plt.legend()
        plt.savefig(os.path.join(output_dir, f"reaction_time_{int(theta0)}deg.png"))
        plt.close()

    plt.figure()
    for name, data in results["robustness"].items():
        t, theta = data["trajectory"]
        plt.plot(t, [math.degrees(th) for th in theta], label=name)
        if data["kick_time"] is not None:
            plt.axvline(data["kick_time"], color="red", linestyle=":")
    plt.axhline(5.0, color="gray", linestyle="--")
    plt.axhline(-5.0, color="gray", linestyle="--")
    plt.xlabel("t (s)")
    plt.ylabel("theta (deg)")
    plt.title("Robustheit: Kraft-Puls-Reaktion")
    plt.legend()
    plt.savefig(os.path.join(output_dir, "robustness_kick.png"))
    plt.close()

    t, theta = results["swingup_capture"]["trajectory"]
    plt.figure()
    plt.plot(t, [math.degrees(th) for th in theta])
    capture_time = results["swingup_capture"]["capture_time"]
    if capture_time is not None:
        plt.axvline(capture_time, color="red", linestyle=":")
    plt.xlabel("t (s)")
    plt.ylabel("theta (deg)")
    plt.title("Swing-up ab realer Anfangsbedingung")
    plt.savefig(os.path.join(output_dir, "swingup_capture.png"))
    plt.close()


if __name__ == "__main__":
    fmu_path = os.path.abspath("InvertedPendulumMB.fmu")
    if not os.path.exists(fmu_path):
        raise SystemExit(
            "InvertedPendulumMB.fmu not found in project root. "
            "Copy the (gitignored) FMU into place before running this benchmark."
        )

    results = run_all_scenarios(fmu_path)

    report = generate_report(results)
    with open("AP3_Reglervergleich.md", "w", encoding="utf-8") as f:
        f.write(report)

    generate_plots(results, "benchmark_plots")

    print("Wrote AP3_Reglervergleich.md and benchmark_plots/")

import pygame
import math
import pandas as pd
from datetime import datetime
import os
from abc import ABC, abstractmethod

from fmpy import read_model_description, extract
from fmpy.fmi2 import FMU2Slave
import shutil

import numpy as np
import scipy.linalg


def redraw(screen, time, dt, score, precision, s, v, taus, phis, auto_mode=False, paused=False, controller_name="PD", difficulty="Standard", hint=None):
    width, height = screen.get_size()
    scale = min(width, height)

    hell = (240, 240, 240)
    dunkel = (64, 64, 64)
    grau = (200, 200, 200)
    blau = (100, 165, 220)
    rot = (240, 85, 70)
    gruen = (60, 200, 80)

    screen.fill(hell)

    def display(string, pos, centered=True, color=(0, 0, 0), size=None):
        font = pygame.font.SysFont(None, size or math.ceil(scale / 32))
        text = font.render(string, True, color)
        if centered:
            w, h = font.size(string)
            screen.blit(text, (pos[0] - w // 2, pos[1] - h // 2))
        else:
            screen.blit(text, (pos[0], pos[1]))

    def plot(values, origin, color, scaling):
        N = min(len(values), 250)
        w = width * (0.5 - 0.02)
        h = height * (0.5 - 0.02)
        pygame.draw.line(screen, dunkel, origin + (0, h / 2), origin - (0, h / 2), 2)
        pygame.draw.line(screen, dunkel, origin, origin + (w, 0), 2)
        if N >= 2:
            xx = [origin.x + (i / N) * w for i in range(N)]
            yy = [origin.y - (val * scaling / 2) * h for val in values[-N:]]
            pygame.draw.lines(screen, color, False, list(zip(xx, yy)), width=3)

    def tachometer(value):
        radius = scale * 0.2
        center = pygame.Vector2(width / 4, height * 0.85)
        max_val = 10
        tacho_val = max(-max_val, min(max_val, value))
        pygame.draw.circle(
            screen,
            "black",
            center,
            radius,
            math.ceil(radius * 0.02),
            draw_top_left=True,
            draw_top_right=True,
        )
        display("0", center + radius * pygame.Vector2(0, -0.6))
        display(str(-max_val), center + radius * pygame.Vector2(-0.6, 0))
        display(str(max_val), center + radius * pygame.Vector2(0.6, 0))
        angle = (1 - tacho_val / max_val) * math.pi / 2
        direction = radius * pygame.Vector2(math.cos(angle), -math.sin(angle))
        orthogonal = radius * pygame.Vector2(-math.sin(angle), -math.cos(angle))
        pygame.draw.polygon(
            screen,
            rot,
            [
                center - 0.03 * orthogonal,
                center + 0.85 * direction - 0.01 * orthogonal,
                center + 0.85 * direction + 0.01 * orthogonal,
                center + 0.03 * orthogonal,
            ],
        )
        pygame.draw.circle(screen, dunkel, center, radius * 0.08)
        display("{: .3f}".format(value), center + radius * pygame.Vector2(0, 0.2))

    def car(s, phi):
        ground = pygame.Vector2(width / 4, height * 0.5)
        thickness = math.ceil(scale * 3e-3)
        ground_w = width * 0.4
        pygame.draw.line(
            screen,
            "black",
            ground + (-ground_w / 2, 0),
            ground + (ground_w / 2, 0),
            thickness,
        )
        N = 6
        for i in range(1, N + 2):
            t = s % 1
            pt = ground + (ground_w * (-0.5 + (i - t) / N), 0)
            pygame.draw.line(
                screen, "black", pt, pt + (-scale * 0.03, scale * 0.03), thickness
            )
        pygame.draw.rect(
            screen,
            hell,
            pygame.Rect(
                ground.x - ground_w / 2 - scale * 0.04,
                ground.y - 1,
                scale * 0.04,
                scale * 0.04,
            ),
        )
        pygame.draw.rect(
            screen,
            hell,
            pygame.Rect(
                ground.x + ground_w / 2, ground.y - 1, ground_w / N + 2, scale * 0.04
            ),
        )
        wheel_r = scale * 0.03
        pygame.draw.circle(
            screen,
            "black",
            ground + (-wheel_r * 2, -wheel_r),
            wheel_r,
            math.ceil(wheel_r / 2),
        )
        pygame.draw.circle(
            screen,
            "black",
            ground + (wheel_r * 2, -wheel_r),
            wheel_r,
            math.ceil(wheel_r / 2),
        )
        car_pos = ground + (0, -5 * wheel_r)
        w, h = 8 * wheel_r, 3.7 * wheel_r
        rect = pygame.Rect(car_pos.x - w / 2, car_pos.y, w, h)
        pygame.draw.rect(screen, grau, rect, border_radius=math.ceil(scale / 256))
        pygame.draw.rect(
            screen, "black", rect, border_radius=math.ceil(scale / 256), width=thickness
        )
        pend_len = scale * 0.15
        pend_r = scale * 0.025
        pend_c = car_pos + pend_len * pygame.Vector2(math.cos(phi), -math.sin(phi))
        pygame.draw.line(screen, dunkel, car_pos, pend_c, 2 * thickness)
        pygame.draw.circle(screen, dunkel, car_pos, 3 * thickness)
        pygame.draw.circle(screen, blau, pend_c, pend_r)
        pygame.draw.circle(screen, "black", pend_c, pend_r, thickness)
        pygame.draw.circle(
            screen, hell, pend_c - (pend_r / 4, pend_r / 4), pend_r * 0.2
        )

    car(s, math.pi / 2 + phis[-1])
    plot(
        [(phi) % (2 * math.pi) - math.pi for phi in phis],
        pygame.Vector2(width * (0.5 + 0.01), height / 4),
        blau,
        0.1,
    )
    tachometer(v)
    plot(taus, pygame.Vector2(width * (0.5 + 0.01), height * 3 / 4), rot, 0.03)

    display("Angle: φ(t)", (3 * width / 4, height * 0.05))
    display("Control: a(t)", (3 * width / 4, height * 0.55))
    display("Velocity: v(t)", (width / 4, height * 0.6))
    display(f"Score: {round(score, 4)}", (width / 6, height * 0.05))
    display("Time: t = {: .1f}".format(time), (width / 3, height * 0.05))

    badge_color = gruen if auto_mode else rot
    badge_label = (
        f"AUTO [{controller_name}]  [H to disable]" if auto_mode else "MANUAL  [H for auto]"
    )
    badge_rect = pygame.Rect(
        width - math.ceil(scale * 0.30),
        math.ceil(scale * 0.02),
        math.ceil(scale * 0.28),
        math.ceil(scale * 0.06),
    )
    pygame.draw.rect(screen, badge_color, badge_rect, border_radius=8)
    pygame.draw.rect(screen, dunkel, badge_rect, 2, border_radius=8)
    display(
        badge_label,
        badge_rect.center,
        color=(255, 255, 255),
        size=math.ceil(scale / 42),
    )

    difficulty_rect = pygame.Rect(
        width - math.ceil(scale * 0.30),
        math.ceil(scale * 0.02) + badge_rect.height + math.ceil(scale * 0.01),
        math.ceil(scale * 0.28),
        math.ceil(scale * 0.06),
    )
    pygame.draw.rect(screen, blau, difficulty_rect, border_radius=8)
    pygame.draw.rect(screen, dunkel, difficulty_rect, 2, border_radius=8)
    display(
        f"Difficulty: {difficulty}  [D]",
        difficulty_rect.center,
        color=(255, 255, 255),
        size=math.ceil(scale / 42),
    )

    if paused:
        pause_rect = pygame.Rect(
            width - math.ceil(scale * 0.30),
            math.ceil(scale * 0.02) + badge_rect.height + difficulty_rect.height + 2 * math.ceil(scale * 0.01),
            math.ceil(scale * 0.28),
            math.ceil(scale * 0.06),
        )
        pygame.draw.rect(screen, dunkel, pause_rect, border_radius=8)
        pygame.draw.rect(screen, dunkel, pause_rect, 2, border_radius=8)
        display(
            "PAUSED  [P to resume]",
            pause_rect.center,
            color=(255, 255, 255),
            size=math.ceil(scale / 42),
        )

    if hint:
        hint_rect = pygame.Rect(
            width // 2 - math.ceil(scale * 0.20),
            math.ceil(scale * 0.02),
            math.ceil(scale * 0.40),
            math.ceil(scale * 0.05),
        )
        pygame.draw.rect(screen, dunkel, hint_rect, border_radius=8)
        display(
            hint,
            hint_rect.center,
            color=(255, 255, 255),
            size=math.ceil(scale / 46),
        )


def _ensure_leaderboard_columns(df):
    """
    Ensure Mode and Difficulty columns exist in the DataFrame.
    Backfills missing columns with sensible defaults for legacy CSVs.
    Returns the modified DataFrame.
    """
    if "Mode" not in df.columns:
        df["Mode"] = "—"
    else:
        df["Mode"] = df["Mode"].fillna("—")
    if "Difficulty" not in df.columns:
        df["Difficulty"] = "Standard"
    else:
        df["Difficulty"] = df["Difficulty"].fillna("Standard")
    return df


def update_leaderboard(score, player_name, mode, difficulty="Standard", filename="leaderboard.csv"):
    now = datetime.now()
    entry = {
        "Date": now.strftime("%Y-%m-%d"),
        "Time": now.strftime("%H:%M:%S"),
        "Name": player_name,
        "Score": round(score, 2),
        "Mode": mode,
        "Difficulty": difficulty,
    }
    df = (
        pd.read_csv(filename)
        if os.path.exists(filename)
        else pd.DataFrame(columns=["Date", "Time", "Name", "Score", "Mode", "Difficulty"])
    )
    df = _ensure_leaderboard_columns(df)
    df = (
        pd.DataFrame([entry])
        if df.empty
        else pd.concat([df, pd.DataFrame([entry])], ignore_index=True)
    )
    df = df.sort_values(by="Score", ascending=False).reset_index(drop=True)
    df.to_csv(filename, index=False)
    print(f"{player_name} achieved score: {score:.2f} - written to {filename}\n")


def overlay_leaderboard(screen, filename="leaderboard.csv", top_n=10):
    df = (
        pd.read_csv(filename).sort_values(by="Score", ascending=False).head(top_n)
        if os.path.exists(filename)
        else pd.DataFrame(columns=["Date", "Time", "Name", "Score", "Mode", "Difficulty"])
    )
    df = _ensure_leaderboard_columns(df)

    title_font = pygame.font.SysFont("arialblack", 64)
    entry_font = pygame.font.SysFont("arial", 36)
    prompt_font = pygame.font.SysFont("arial", 28)

    overlay = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
    overlay.fill((0, 0, 0, 50))
    panel_w, panel_h = 1000, 800
    panel_x = (screen.get_width() - panel_w) // 2
    panel_y = (screen.get_height() - panel_h) // 2
    panel = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
    panel.fill((30, 30, 30, 15))
    pygame.draw.rect(panel, (40, 40, 40, 100), panel.get_rect(), border_radius=20)
    title = title_font.render("Inverted Pendulum", True, (255, 255, 255))
    panel.blit(title, (panel_w // 2 - title.get_width() // 2, 0))
    for i, row in df.iterrows():
        rendered = entry_font.render(
            f"{i+1:>2}. {row['Name']:10} {row['Score']:.2f}  [{row['Mode']} / {row['Difficulty']}]",
            True,
            (255, 220, 180),
        )
        panel.blit(rendered, (60, 120 + i * 35))
    prompt = prompt_font.render("Press any key to start...", True, (180, 180, 255))
    panel.blit(prompt, (panel_w // 2 - prompt.get_width() // 2, panel_h - 60))
    overlay.blit(panel, (panel_x, panel_y))
    screen.blit(overlay, (0, 0))
    pygame.display.flip()

    waiting = True
    while waiting:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                waiting = False


def get_player_name(screen):
    font = pygame.font.SysFont("arial", 40)
    prompt = font.render("Enter your name:", True, (255, 255, 255))
    clock = pygame.time.Clock()
    name = ""
    while True:
        screen.fill((0, 0, 0))
        screen.blit(prompt, (screen.get_width() // 2 - prompt.get_width() // 2, 150))
        surf = font.render(name, True, (180, 255, 180))
        bw, bh = 400, 70
        bx = screen.get_width() // 2 - bw // 2
        pygame.draw.rect(screen, (50, 50, 50), (bx, 220, bw, bh))
        screen.blit(surf, (bx + 10, 220 + (bh - surf.get_height()) // 2))
        pygame.display.flip()
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RETURN and name.strip():
                    return name.strip()
                elif event.key == pygame.K_BACKSPACE:
                    name = name[:-1]
                elif len(name) < 15:
                    name += event.unicode

# abstract controller base
class Controller(ABC):
    @abstractmethod
    def compute(self, phi_fmu, vphi, s, v):
        pass

# override controller with simple controller
class SimpleController(Controller):
    K_PHI = 25
    K_VPHI = 20
    MAX_TAU = 10.0

    def compute(self, phi_fmu, vphi, s, v):
        # Upright is phi = pi; normalize error around that
        phi_err = ((phi_fmu) % (2 * math.pi) - math.pi)

        tau = -(
            +self.K_PHI * phi_err + self.K_VPHI * vphi
        )

        return max(-self.MAX_TAU, min(self.MAX_TAU, tau))

# TODO: some other controllers

def compute_lqr_gain(M, m, l, g, d_cart, d_pend, Q, R):
    A = np.array([
        [0, 1, 0, 0],
        [0, -d_cart / M, m * g / M, -d_pend / (M * l)],
        [0, 0, 0, 1],
        [0, -d_cart / (l * M), (M + m) * g / (l * M), -(M + m) * d_pend / (m * l**2 * M)],
    ])
    B = np.array([[0], [1 / M], [0], [1 / (l * M)]])
    P = scipy.linalg.solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K


class LQRController(Controller):
    # Must match InvertedPendulumMB.mo's parameters (m_cart, m_pend, l, d_cart, d_pend) — no automatic sync.
    M = 5
    m = 0.5
    l = 0.5
    g = 9.81
    d_cart = 0.15
    d_pend = 0.01
    Q = np.diag([1.0, 1.0, 10.0, 1.0])
    R = np.array([[1.0]])
    MAX_TAU = 10.0

    def __init__(self):
        self.K = compute_lqr_gain(
            self.M, self.m, self.l, self.g, self.d_cart, self.d_pend, self.Q, self.R
        )

    def compute(self, phi_fmu, vphi, s, v):
        theta = (phi_fmu % (2 * math.pi)) - math.pi
        x = np.array([[s], [v], [theta], [vphi]])
        tau = (-self.K @ x).item()
        return max(-self.MAX_TAU, min(self.MAX_TAU, tau))


def pendulum_energy(m, l, g, phi, vphi):
    return 0.5 * m * l**2 * vphi**2 + m * g * l * (1 - math.cos(phi))


class SwingUpController(Controller):
    # Must match InvertedPendulumMB.mo's parameters (m_cart, m_pend, l, d_cart, d_pend) — no automatic sync.
    M = 5
    m = 0.5
    l = 0.5
    g = 9.81
    d_cart = 0.15
    d_pend = 0.01
    MAX_TAU = 10.0

    # Tuning constants for the real start condition (phi0=67.5 deg, see
    # InvertedPendulumMB.mo); adjustable if interactive testing shows they need retuning.
    # These (K_ENERGY and the hysteresis thresholds below) are tuned/verified for the
    # Euler-solver FMU (InvertedPendulumMB.fmu, the one the shipped game uses) and stay
    # the defaults here. A swept comparison against the Linux CVODE FMU
    # (InvertedPendulumMB_cvode_linux.fmu, WSL-only, see CLAUDE.md backlog
    # "CVODE-Solver-Vergleich via WSL") found K_ENERGY=25.0 + CAPTURE_THETA=5deg capture
    # ~6% faster under CVODE (5.84s -> 5.48s from the real start condition) before
    # plateauing/reversing (K_ENERGY beyond ~40 has no further effect: tau saturates at
    # MAX_TAU regardless; CAPTURE_THETA beyond ~15deg actively hurts, since switching to
    # LQR too early causes a false start that falls back to swing-up via the
    # RELEASE_THETA hysteresis). CAPTURE_VPHI had no measurable effect in [1.0, 3.0] for
    # this trajectory. The remaining gap to Euler's 4.20s is actuator-saturation-bound,
    # not closable by retuning. Pass overrides via the constructor (e.g.
    # PENDULUM_SWINGUP_K_ENERGY) when running against the CVODE FMU; never change these
    # defaults, which is what the shipped/graded game uses.
    K_ENERGY = 10.0
    CAPTURE_THETA = math.radians(10)
    CAPTURE_VPHI = 2.0
    RELEASE_THETA = math.radians(25)

    def __init__(self, k_energy=None, capture_theta=None, capture_vphi=None, release_theta=None):
        self.lqr = LQRController()
        self.mode = "swingup"
        self.k_energy = self.K_ENERGY if k_energy is None else k_energy
        self.capture_theta = self.CAPTURE_THETA if capture_theta is None else capture_theta
        self.capture_vphi = self.CAPTURE_VPHI if capture_vphi is None else capture_vphi
        self.release_theta = self.RELEASE_THETA if release_theta is None else release_theta

    def compute(self, phi_fmu, vphi, s, v):
        theta = (phi_fmu % (2 * math.pi)) - math.pi

        if self.mode == "swingup" and abs(theta) < self.capture_theta and abs(vphi) < self.capture_vphi:
            self.mode = "lqr"
        elif self.mode == "lqr" and abs(theta) > self.release_theta:
            self.mode = "swingup"

        if self.mode == "lqr":
            return self.lqr.compute(phi_fmu, vphi, s, v)

        energy = pendulum_energy(self.m, self.l, self.g, phi_fmu, vphi)
        energy_top = 2 * self.m * self.g * self.l
        sign = 1.0 if math.cos(phi_fmu) * vphi >= 0 else -1.0
        a_cmd = self.k_energy * (energy - energy_top) * sign
        tau = (
            a_cmd * (self.M + self.m * math.sin(phi_fmu) ** 2)
            + self.d_cart * v
            - self.m * self.l * math.sin(phi_fmu) * vphi**2
            - self.m * self.g * math.sin(phi_fmu) * math.cos(phi_fmu)
            - (self.d_pend / self.l) * math.cos(phi_fmu) * vphi
        )
        return max(-self.MAX_TAU, min(self.MAX_TAU, tau))


DIFFICULTY_ORDER = ("Leicht", "Standard", "Schwer")

DIFFICULTY_LEVELS = {
    "Leicht": {
        "bonus_zone_deg": 20.0,
        "tight_bonus_zone_deg": 8.0,
        "max_angle_deg": 90.0,
        "m_cart": 5.0,
        "m_pend": 0.3,
        "d_cart": 0.08,
        "d_pend": 0.01,
    },
    "Standard": {
        "bonus_zone_deg": 15.0,
        "tight_bonus_zone_deg": 5.0,
        "max_angle_deg": 90.0,
        "m_cart": 5.0,
        "m_pend": 0.5,
        "d_cart": 0.15,
        "d_pend": 0.01,
    },
    "Schwer": {
        "bonus_zone_deg": 8.0,
        "tight_bonus_zone_deg": 3.0,
        # Narrower than Leicht/Standard's 90 deg: at 90 deg, a "naive swing" policy
        # (fixed-period bang-bang, no attempt to balance) resonates with Schwer's
        # heavier/less-damped pendulum and scores MORE than the same naive policy on
        # Leicht (228 vs 233 at each difficulty's own resonance period -- roughly tied,
        # not the intended "Schwer requires more skill") purely by sweeping the wide
        # outer scoring cone, without any real balancing skill. Narrowing to 40 deg
        # drops Schwer's best naive-swing score to ~111, clearly below both Standard's
        # ~125 and Leicht's ~233, restoring the intended skill/difficulty ordering.
        # Legitimate precise play (bonus_zone=8deg, tight_bonus_zone=3deg) is far
        # inside 40 deg and essentially unaffected.
        "max_angle_deg": 40.0,
        "m_cart": 5.0,
        "m_pend": 0.9,
        "d_cart": 0.05,
        "d_pend": 0.01,
    },
}


def next_difficulty(current):
    idx = DIFFICULTY_ORDER.index(current)
    return DIFFICULTY_ORDER[(idx + 1) % len(DIFFICULTY_ORDER)]


K_STABILITY = 0.5


def compute_score_increment(
    angle, stable_streak, bonus_zone=math.radians(15), tight_bonus_zone=math.radians(5), max_angle=math.pi / 2
):
    increment = 0.0
    if angle <= max_angle:
        closeness = (max_angle - angle) / max_angle
        increment += 0.1 + 0.2 * closeness

        if angle <= bonus_zone:
            close2 = (bonus_zone - angle) / bonus_zone
            increment += 2 * (close2**2)

        if angle <= tight_bonus_zone:
            close3 = (tight_bonus_zone - angle) / tight_bonus_zone
            increment += 3 * (close3**2)

    increment += K_STABILITY * stable_streak
    return increment


def classify_mode(auto_time, manual_time):
    if manual_time <= 0.0 and auto_time > 0.0:
        return "Auto"
    if auto_time <= 0.0 and manual_time > 0.0:
        return "Manual"
    return "Mixed"


def controller_display_name(controller, name):
    if not hasattr(controller, "mode"):
        return name
    submode = "s" if controller.mode == "swingup" else "b"
    return f"SU:{submode}"


def apply_difficulty_physics(fmu, value_refs, difficulty):
    level = DIFFICULTY_LEVELS[difficulty]
    fmu.setReal(
        [value_refs["m_cart"], value_refs["m_pend"], value_refs["d_cart"], value_refs["d_pend"]],
        [level["m_cart"], level["m_pend"], level["d_cart"], level["d_pend"]],
    )


def reset_round(fmu, value_refs, difficulty, s_ref, v_ref, phi_ref, vphi_ref):
    fmu.reset()
    fmu.setupExperiment(startTime=0.0)
    apply_difficulty_physics(fmu, value_refs, difficulty)
    fmu.enterInitializationMode()
    fmu.exitInitializationMode()
    return (
        fmu.getReal([s_ref])[0],
        fmu.getReal([v_ref])[0],
        fmu.getReal([phi_ref])[0],
        fmu.getReal([vphi_ref])[0],
    )


def run_game(screen):
    # Overridable for the WSL/CVODE solver-comparison spike (CLAUDE.md backlog) —
    # unset, this is byte-identical to the shipped Windows/Euler default.
    fmu_path = os.path.abspath(os.environ.get("PENDULUM_FMU", "InvertedPendulumMB.fmu"))
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

    difficulty = "Standard"
    value_refs = {
        "m_cart": ref("m_cart"),
        "m_pend": ref("m_pend"),
        "d_cart": ref("d_cart"),
        "d_pend": ref("d_pend"),
    }
    apply_difficulty_physics(fmu, value_refs, difficulty)

    fmu.enterInitializationMode()
    fmu.exitInitializationMode()

    tau_ref = ref("tau")
    s_ref = ref("s")
    v_ref = ref("v")
    phi_ref = ref("phi")
    vphi_ref = ref("vphi")

    dt = 0.02
    # Explicit-Euler FMU co-simulation at the full 0.02s step can numerically
    # inject energy into a lightly-damped oscillator (confirmed: at the low
    # d_pend used since AP3 Teil 2's swing-up fix, single 0.02s steps make
    # free-swinging pendulum motion diverge instead of decay). Sub-stepping
    # the FMU call keeps physics accurate without changing the 50Hz game
    # timing, scoring cadence, or actuation resolution (tau is still set
    # once per outer frame, before the substep loop).
    SUBSTEPS = 10
    GAME_DURATION = 40
    MAX_TAU = 10.0
    time = 0.0
    score = 0.0
    auto_mode = False
    paused = False
    stable_streak = 0.0
    hint_text = None
    hint_frames_left = 0

    s = 0.0
    v = 0.0
    phi = math.pi + 0.75 * math.pi / 2
    vphi = 0.0

    auto_time = 0.0
    manual_time = 0.0

    taus, phis = [], []

    def _env_override(name):
        value = os.environ.get(name)
        return None if value is None else float(value)

    swingup_k_energy = _env_override("PENDULUM_SWINGUP_K_ENERGY")
    swingup_capture_theta_deg = _env_override("PENDULUM_SWINGUP_CAPTURE_THETA_DEG")
    swingup_capture_theta = None if swingup_capture_theta_deg is None else math.radians(swingup_capture_theta_deg)
    controllers = {
        "PD": SimpleController(),
        "LQR": LQRController(),
        "SwingUp": SwingUpController(k_energy=swingup_k_energy, capture_theta=swingup_capture_theta),
    }
    controller_name = "PD"
    clock = pygame.time.Clock()

    redraw(
        screen, time, dt, 0, 0.25, s, v, [phi], [vphi], auto_mode, paused,
        controller_display_name(controllers[controller_name], controller_name),
        difficulty=difficulty, hint=hint_text,
    )
    pygame.display.flip()
    overlay_leaderboard(screen)

    while time < GAME_DURATION:
        keys = pygame.key.get_pressed()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
            if event.type == pygame.KEYDOWN and event.key == pygame.K_h:
                if difficulty == "Standard":
                    auto_mode = not auto_mode
                else:
                    hint_text = "Auto nur bei Standard"
                    hint_frames_left = 90
            if event.type == pygame.KEYDOWN and event.key == pygame.K_p:
                paused = not paused
            if event.type == pygame.KEYDOWN and event.key == pygame.K_l:
                names = list(controllers)
                controller_name = names[(names.index(controller_name) + 1) % len(names)]
            if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                s, v, phi, vphi = reset_round(fmu, value_refs, difficulty, s_ref, v_ref, phi_ref, vphi_ref)
                time = 0.0
                score = 0.0
                stable_streak = 0.0
                auto_time = 0.0
                manual_time = 0.0
                taus, phis = [], []
            if event.type == pygame.KEYDOWN and event.key == pygame.K_d:
                if auto_mode:
                    hint_text = "Nur im Manual-Modus änderbar"
                    hint_frames_left = 90
                else:
                    difficulty = next_difficulty(difficulty)
                    s, v, phi, vphi = reset_round(fmu, value_refs, difficulty, s_ref, v_ref, phi_ref, vphi_ref)
                    time = 0.0
                    score = 0.0
                    stable_streak = 0.0
                    auto_time = 0.0
                    manual_time = 0.0
                    taus, phis = [], []
            if event.type == pygame.KEYDOWN and event.key == pygame.K_F12:
                os.makedirs("screenshots", exist_ok=True)
                screenshot_path = os.path.join(
                    "screenshots", f"pendulum_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
                )
                pygame.image.save(screen, screenshot_path)
                hint_text = f"Screenshot gespeichert: {screenshot_path}"
                hint_frames_left = 90

        if hint_frames_left > 0:
            hint_frames_left -= 1
            if hint_frames_left == 0:
                hint_text = None

        if paused:
            plot_taus = taus if taus else [0.0]
            plot_phis = phis if phis else [phi - math.pi]
            redraw(
                screen, time, dt, score, 0.25, s, v, plot_taus, plot_phis, auto_mode, paused,
                controller_display_name(controllers[controller_name], controller_name),
                difficulty=difficulty, hint=hint_text,
            )
            pygame.display.flip()
            clock.tick(60)
            continue

        if auto_mode:
            tau = controllers[controller_name].compute(phi, vphi, s, v)
            auto_time += dt
        else:
            tau = 0.0
            if keys[pygame.K_LEFT]:
                tau = -MAX_TAU
            if keys[pygame.K_RIGHT]:
                tau = MAX_TAU
            manual_time += dt

        fmu.setReal([tau_ref], [tau])
        substep_dt = dt / SUBSTEPS
        for _ in range(SUBSTEPS):
            time += substep_dt
            fmu.doStep(currentCommunicationPoint=time, communicationStepSize=substep_dt)

        s = fmu.getReal([s_ref])[0]
        v = fmu.getReal([v_ref])[0]
        phi = fmu.getReal([phi_ref])[0]
        vphi = fmu.getReal([vphi_ref])[0]

        angle = (phi - math.pi) % (2 * math.pi)
        if angle > math.pi:
            angle -= 2 * math.pi
        angle = abs(angle)

        level = DIFFICULTY_LEVELS[difficulty]
        bonus_zone = math.radians(level["bonus_zone_deg"])
        tight_bonus_zone = math.radians(level["tight_bonus_zone_deg"])
        max_angle = math.radians(level["max_angle_deg"])

        if angle <= tight_bonus_zone:
            stable_streak += dt
        else:
            stable_streak = 0.0

        score += compute_score_increment(
            angle, stable_streak, bonus_zone=bonus_zone, tight_bonus_zone=tight_bonus_zone, max_angle=max_angle
        )

        phis.append(phi - math.pi)
        taus.append(tau)
        if len(phis) > 750:
            phis.pop(0)
            taus.pop(0)

        redraw(
            screen, time, dt, score, 0.25, s, v, taus, phis, auto_mode, paused,
            controller_display_name(controllers[controller_name], controller_name),
            difficulty=difficulty, hint=hint_text,
        )
        pygame.display.flip()
        clock.tick(60)

    fmu.terminate()
    fmu.freeInstance()
    shutil.rmtree(unzipdir)
    return score, classify_mode(auto_time, manual_time), difficulty


def main():
    pygame.init()
    screen = pygame.display.set_mode((1920, 1080), pygame.RESIZABLE)
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    pygame.mouse.set_visible(False)
    pygame.event.set_grab(True)
    pygame.key.set_mods(0)
    pygame.mouse.set_visible(True)

    while True:
        score, mode, difficulty = run_game(screen)
        player_name = get_player_name(screen)
        update_leaderboard(score, player_name, mode, difficulty)


if __name__ == "__main__":
    main()

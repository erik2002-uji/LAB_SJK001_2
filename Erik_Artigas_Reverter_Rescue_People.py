import cv2
import numpy as np
import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto

import WebGUI as GUI
import HAL


@dataclass
class MissionConfig:
    center_x: float = 33.0
    center_y: float = -35.0
    cruise_alt: float = 4.0
    spiral_speed: float = 2.0
    spiral_growth: float = 0.2
    spiral_r0: float = 0.10
    max_victims: int = 6
    victim_merge_radius: float = 4.0
    kp_xy: float = 1.0
    kp_z: float = 0.5
    max_xy_speed: float = 2.5
    max_travel_speed: float = 3.0
    rotate_step_deg: int = 30
    cascade_scale: float = 1.2
    cascade_neighbors: int = 3


class Phase(Enum):
    TAKEOFF = auto()
    GOTO_CENTER = auto()
    SEARCH = auto()
    REPORT = auto()
    RETURN_HOME = auto()
    LAND = auto()


@dataclass
class SpiralState:
    theta: float = 0.0


@dataclass
class MissionState:
    phase: Phase = Phase.TAKEOFF
    spiral: SpiralState = field(default_factory=SpiralState)
    victims_xy: list = field(default_factory=list)
    last_tick: float = field(default_factory=time.time)


def clamp(value: float, vmin: float, vmax: float) -> float:
    return max(min(value, vmax), vmin)


def dist2d(ax: float, ay: float, bx: float, by: float) -> float:
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)


def rotate_gray(img_gray: np.ndarray, deg: float) -> np.ndarray:
    if deg == 0:
        return img_gray
    h, w = img_gray.shape[:2]
    center = (w / 2.0, h / 2.0)
    M = cv2.getRotationMatrix2D(center, deg, 1.0)
    return cv2.warpAffine(img_gray, M, (w, h), flags=cv2.INTER_LINEAR)


class FaceDetector:
    def __init__(self, scale_factor: float, min_neighbors: int):
        cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        self.cascade = cv2.CascadeClassifier(cascade_path)
        self.scale_factor = scale_factor
        self.min_neighbors = min_neighbors

    def any_face(self, gray: np.ndarray, rotate_step_deg: int) -> bool:
        for deg in range(0, 360, rotate_step_deg):
            img = rotate_gray(gray, deg)
            faces = self.cascade.detectMultiScale(
                img, self.scale_factor, self.min_neighbors
            )
            if len(faces) > 0:
                return True
        return False


def compute_dt(state: MissionState) -> float:
    now = time.time()
    dt = now - state.last_tick
    state.last_tick = now
    return max(dt, 0.01)


def handle_takeoff(cfg: MissionConfig, ms: MissionState, z: float):
    HAL.takeoff()
    if z < (cfg.cruise_alt - 0.2):
        HAL.set_cmd_vel(0.0, 0.0, 1.5, 0.0)
    else:
        ms.phase = Phase.GOTO_CENTER


def handle_goto_center(cfg: MissionConfig, ms: MissionState, x: float, y: float, z: float):
    dx = cfg.center_x - x
    dy = cfg.center_y - y
    d = math.sqrt(dx * dx + dy * dy)
    vz = (cfg.cruise_alt - z) * cfg.kp_z

    if d < 1.0:
        HAL.set_cmd_vel(0.0, 0.0, 0.0, 0.0)
        ms.phase = Phase.SEARCH
        return

    vx = clamp(dx * 0.5, -cfg.max_travel_speed, cfg.max_travel_speed)
    vy = clamp(dy * 0.5, -cfg.max_travel_speed, cfg.max_travel_speed)
    HAL.set_cmd_vel(vx, vy, vz, 0.0)


def spiral_target(cfg: MissionConfig, ms: MissionState, dt: float):
    r = cfg.spiral_r0 + cfg.spiral_growth * ms.spiral.theta
    r = max(r, cfg.spiral_r0)
    omega = cfg.spiral_speed / r
    ms.spiral.theta += omega * dt
    tx = cfg.center_x + r * math.cos(ms.spiral.theta)
    ty = cfg.center_y + r * math.sin(ms.spiral.theta)
    return tx, ty, r


def register_victim_if_new(cfg: MissionConfig, ms: MissionState, x: float, y: float):
    for vx, vy in ms.victims_xy:
        if dist2d(x, y, vx, vy) < cfg.victim_merge_radius:
            return False
    ms.victims_xy.append([x, y])
    return True


def handle_search(cfg: MissionConfig, ms: MissionState, x: float, y: float, z: float, dt: float):
    if len(ms.victims_xy) >= cfg.max_victims:
        HAL.set_cmd_vel(0.0, 0.0, 0.0, 0.0)
        ms.phase = Phase.REPORT
        return

    tx, ty, _ = spiral_target(cfg, ms, dt)
    ex = tx - x
    ey = ty - y
    vx = clamp(ex * cfg.kp_xy, -cfg.max_xy_speed, cfg.max_xy_speed)
    vy = clamp(ey * cfg.kp_xy, -cfg.max_xy_speed, cfg.max_xy_speed)
    vz = (cfg.cruise_alt - z) * cfg.kp_z
    HAL.set_cmd_vel(vx, vy, vz, 0.0)


def handle_report(cfg: MissionConfig, ms: MissionState):
    print(f"Victimas detectadas: {len(ms.victims_xy)}")
    for i, (vx, vy) in enumerate(ms.victims_xy, start=1):
        print(f"{i}: {vx:.2f}, {vy:.2f}")
    ms.phase = Phase.RETURN_HOME


def handle_return_home(cfg: MissionConfig, ms: MissionState):
    HAL.set_cmd_pos(0.0, 0.0, cfg.cruise_alt, 0.0)
    ms.phase = Phase.LAND


def handle_land(ms: MissionState, x: float, y: float):
    if math.sqrt(x * x + y * y) < 1.0:
        HAL.land()
        return True
    return False


def main():
    cfg = MissionConfig()
    ms = MissionState()
    detector = FaceDetector(cfg.cascade_scale, cfg.cascade_neighbors)

    while True:
        dt = compute_dt(ms)

        img = HAL.get_ventral_image()
        if img is None:
            continue

        x, y, z = HAL.get_position()

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        has_face = detector.any_face(gray, cfg.rotate_step_deg)

        if has_face and ms.phase == Phase.SEARCH:
            if register_victim_if_new(cfg, ms, x, y):
                h, w = img.shape[:2]
                cv2.circle(img, (w // 2, h // 2), 30, (0, 255, 0), 3)

        GUI.showImage(img)

        if ms.phase == Phase.TAKEOFF:
            handle_takeoff(cfg, ms, z)

        elif ms.phase == Phase.GOTO_CENTER:
            handle_goto_center(cfg, ms, x, y, z)

        elif ms.phase == Phase.SEARCH:
            handle_search(cfg, ms, x, y, z, dt)

        elif ms.phase == Phase.REPORT:
            handle_report(cfg, ms)

        elif ms.phase == Phase.RETURN_HOME:
            handle_return_home(cfg, ms)

        elif ms.phase == Phase.LAND:
            if handle_land(ms, x, y):
                break


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import argparse
import importlib
import pathlib
import time

import mujoco


TARGET_QPOS = [0.0, -0.55, 1.10, -0.65, 0.28, 0.45]
MODEL_FILE = "robotic_arm_scene.xml"


def build_target_trajectory(alpha: float):
    alpha = min(max(alpha, 0.0), 1.0)
    # Smoothstep interpolation avoids abrupt command jumps.
    s = alpha * alpha * (3.0 - 2.0 * alpha)
    return [s * q for q in TARGET_QPOS]


def run_headless(model, data, sim_time_s: float = 6.0):
    start = time.time()
    while data.time < sim_time_s:
        phase = min(data.time / 2.0, 1.0)
        data.ctrl[:] = build_target_trajectory(phase)
        mujoco.mj_step(model, data)

    elapsed = time.time() - start
    print(f"[MuJoCo] Finished headless simulation in {elapsed:.3f}s")
    print("[MuJoCo] Final qpos:", [round(x, 4) for x in data.qpos.tolist()])


def run_with_viewer(model, data, sim_time_s: float = 8.0):
    mujoco_viewer = importlib.import_module("mujoco.viewer")

    with mujoco_viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
        wall_t0 = time.time()
        while viewer.is_running() and data.time < sim_time_s:
            phase = min(data.time / 2.0, 1.0)
            data.ctrl[:] = build_target_trajectory(phase)
            mujoco.mj_step(model, data)
            viewer.sync()

            # Keep real-time pace for easier visual validation.
            sleep_time = (data.time - (time.time() - wall_t0))
            if sleep_time > 0:
                time.sleep(sleep_time)

    print("[MuJoCo] Viewer session finished.")
    print("[MuJoCo] Final qpos:", [round(x, 4) for x in data.qpos.tolist()])


def main():
    parser = argparse.ArgumentParser(description="Phase-1 fixed-pose MuJoCo validation")
    parser.add_argument("--no-viewer", action="store_true", help="Run simulation without GUI")
    parser.add_argument("--gravity-off", action="store_true", help="Disable gravity for pure angle validation")
    args = parser.parse_args()

    model_path = pathlib.Path(__file__).resolve().parent / MODEL_FILE
    if not model_path.exists():
        raise FileNotFoundError(f"Model file not found: {model_path}")

    model = mujoco.MjModel.from_xml_path(str(model_path))
    if args.gravity_off:
        model.opt.gravity[:] = [0.0, 0.0, 0.0]
    data = mujoco.MjData(model)

    if args.no_viewer:
        run_headless(model, data)
    else:
        run_with_viewer(model, data)


if __name__ == "__main__":
    main()

import gradio as gr
import subprocess
import os
import time
import sys

# --- Paths ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BUILD_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "build"))

BIN_PATH = os.path.join(BUILD_DIR, "acc_simulator")
CSV_PATH = os.path.join(BUILD_DIR, "acc_sim.csv")
VIDEO_PATH = os.path.join(BUILD_DIR, "acc_animation_enhanced.mp4")
VIS_SCRIPT = os.path.join(SCRIPT_DIR, "visualization.py")

# --- Ensure permissions ---
if os.path.exists(BIN_PATH):
    os.chmod(BIN_PATH, 0o755)


# --- Helper: Run C++ Simulation ---
def run_simulation(v_set, headway, mu, sim_time, init_gap, lead_accel,
                   v_ego0, v_lead0,
                   Kp_c, Ki_c, Kd_c, Kp_h, Ki_h, Kd_h):

    if not os.path.exists(BIN_PATH):
        return "❌ C++ binary not found! Please build acc_simulator first.", None

    sim_time = min(sim_time, 30)  # limit runtime (e.g. Hugging Face)

    cmd = [
        BIN_PATH, str(v_set), str(headway), str(mu), str(sim_time),
        str(init_gap), str(lead_accel),
        str(v_ego0), str(v_lead0),
        str(Kp_c), str(Ki_c), str(Kd_c),
        str(Kp_h), str(Ki_h), str(Kd_h)
    ]

    try:
        start = time.time()
        subprocess.run(cmd, cwd=BUILD_DIR, check=True, capture_output=True)
        duration = time.time() - start
    except subprocess.CalledProcessError as e:
        return f"❌ Simulation error:\n{e.stderr.decode(errors='ignore')}", None

    if not os.path.exists(CSV_PATH):
        return "❌ Simulation failed — acc_sim.csv not generated!", None

    return f"✅ Simulation completed in {duration:.1f}s", CSV_PATH


# --- Helper: Run Visualization ---
def run_visualization():
    if not os.path.exists(CSV_PATH):
        return "❌ No CSV found. Run simulation first!", None

    try:
        start = time.time()
        subprocess.run(
            [sys.executable, VIS_SCRIPT],
            cwd=SCRIPT_DIR,
            check=True,
            capture_output=True
        )
        duration = time.time() - start
    except subprocess.CalledProcessError as e:
        return f"❌ Visualization failed:\n{e.stderr.decode(errors='ignore')}", None

    if os.path.exists(VIDEO_PATH):
        return f"✅ Visualization complete in {duration:.1f}s", VIDEO_PATH
    else:
        return "⚠ Visualization finished but video not found.", None


# --- Combined Pipeline ---
def simulate_and_visualize(v_set, headway, mu, sim_time, init_gap, lead_accel,
                           v_ego0, v_lead0,
                           Kp_c, Ki_c, Kd_c, Kp_h, Ki_h, Kd_h):

    sim_status, csv_path = run_simulation(v_set, headway, mu, sim_time,
                                          init_gap, lead_accel,
                                          v_ego0, v_lead0,
                                          Kp_c, Ki_c, Kd_c, Kp_h, Ki_h, Kd_h)
    if not csv_path:
        return sim_status, None, None, None

    vis_status, video_path = run_visualization()
    if not video_path:
        return f"{sim_status}\n{vis_status}", csv_path, None, None

    return f"{sim_status}\n{vis_status}", csv_path, video_path, video_path


# --- Gradio Interface ---
with gr.Blocks(title="Adaptive Cruise Control (C++ + Gradio)") as demo:
    gr.Markdown("""
    # 🚗 Adaptive Cruise Control Simulator — PID Tuning + Visualization  
    Tune PID gains, initial conditions, and visualize ego-lead interactions with realistic car models.
    """)

    with gr.Row():
        v_set = gr.Slider(10, 35, value=20, step=0.5, label="Set Speed [m/s]")
        headway = gr.Slider(0.8, 2.5, value=1.2, step=0.1, label="Time Headway [s]")
        mu = gr.Slider(0.3, 1.0, value=0.9, step=0.05, label="Road Friction μ")
        sim_time = gr.Slider(5, 60, value=20, step=5, label="Simulation Time [s]")

    with gr.Row():
        init_gap = gr.Slider(10, 100, value=30, step=1, label="Initial Distance [m]")
        lead_accel = gr.Slider(-2, 2, value=-0.5, step=0.1, label="Lead Acceleration [m/s²]")

    with gr.Row():
        v_ego0 = gr.Slider(0, 35, value=15, step=0.5, label="Ego Initial Speed [m/s]")
        v_lead0 = gr.Slider(0, 35, value=15, step=0.5, label="Lead Initial Speed [m/s]")

    gr.Markdown("### 🧠 PID Gains — Cruise (Speed) Control")
    with gr.Row():
        Kp_c = gr.Number(value=0.4, label="Kp_c")
        Ki_c = gr.Number(value=0.05, label="Ki_c")
        Kd_c = gr.Number(value=0.01, label="Kd_c")

    gr.Markdown("### 📏 PID Gains — Headway (Spacing) Control")
    with gr.Row():
        Kp_h = gr.Number(value=0.6, label="Kp_h")
        Ki_h = gr.Number(value=0.08, label="Ki_h")
        Kd_h = gr.Number(value=0.02, label="Kd_h")

    run_btn = gr.Button("▶ Run Simulation & Visualization")

    status = gr.Textbox(label="Status / Logs", lines=3)
    csv_file = gr.File(label="📄 Download CSV Log")
    video = gr.Video(label="🎬 Simulation Animation", autoplay=True)
    mp4_file = gr.File(label="⬇ Download MP4 Video")

    run_btn.click(
        simulate_and_visualize,
        inputs=[v_set, headway, mu, sim_time, init_gap, lead_accel,
                v_ego0, v_lead0,
                Kp_c, Ki_c, Kd_c, Kp_h, Ki_h, Kd_h],
        outputs=[status, csv_file, video, mp4_file],
    )

demo.launch(share=True, allowed_paths=[BUILD_DIR])


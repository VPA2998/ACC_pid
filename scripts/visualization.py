import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.image as mpimg
import numpy as np
import os

# --- Paths ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BUILD_DIR = os.path.join(SCRIPT_DIR, "..", "build")
BUILD_DIR = os.path.abspath(BUILD_DIR)

CSV_PATH = os.path.join(BUILD_DIR, "acc_sim.csv")
VIDEO_PATH = os.path.join(BUILD_DIR, "acc_animation_enhanced.mp4")

# --- Load simulation data ---
if not os.path.exists(CSV_PATH):
    raise FileNotFoundError(f"❌ Cannot find {CSV_PATH}. Run the simulation first!")

df = pd.read_csv(CSV_PATH)

# --- Handle ACC mode column ---
if "acc_mode" in df.columns:
    df["ACC_mode"] = df["acc_mode"]
    df.drop(columns=["acc_mode"], inplace=True)
elif "ACC_mode" not in df.columns:
    df["ACC_mode"] = "Cruise"

# --- Figure setup ---
fig, axs = plt.subplots(3, 1, figsize=(12, 9), gridspec_kw={"height_ratios": [2.2, 1, 1]})
fig.suptitle("Adaptive Cruise Control — Ego vs Lead Visualization", fontsize=15, fontweight="bold")


# --- Lane setup ---
lane_length = max(df["lead_x"].max(), 100)
axs[0].set_xlim(0, lane_length)
axs[0].set_ylim(-1.5, 1.5)
axs[0].set_xlabel("Distance along road [m]")
axs[0].set_yticks([])
axs[0].set_aspect("auto")

# --- Road background ---
road = plt.Rectangle((0, -1), lane_length, 2, color="gray", alpha=0.3, zorder=0)
axs[0].add_patch(road)

# --- Lane markings ---
for i in np.arange(0, lane_length, 5):
    axs[0].plot([i, i + 2.5], [0.0, 0.0], color="white", lw=2, alpha=0.8, zorder=1)

# --- Load car PNG images ---
ego_img_path = os.path.join(SCRIPT_DIR, "ego_car.png")
lead_img_path = os.path.join(SCRIPT_DIR, "lead_car.png")

if not (os.path.exists(ego_img_path) and os.path.exists(lead_img_path)):
    raise FileNotFoundError("❌ Missing car images! Ensure ego_car.png and lead_car.png exist in scripts/.")

ego_img = mpimg.imread(ego_img_path)
lead_img = mpimg.imread(lead_img_path)

# --- Compute aspect ratios for scaling ---
def get_image_ratio(img):
    h, w = img.shape[:2]
    return w / h

ego_ratio = get_image_ratio(ego_img)
lead_ratio = get_image_ratio(lead_img)

# --- Define real-world car sizes (meters) ---
car_length = 4.5
car_height_ego = car_length / (ego_ratio * 5.0)
car_height_lead = car_length / (lead_ratio * 5.0)

# --- Vertical offset so cars sit on the road ---
y_offset = -0.35

# --- Initialize car image artists ---
ego_artist = axs[0].imshow(
    ego_img,
    extent=[0, car_length,
            y_offset - car_height_ego / 2,
            y_offset + car_height_ego / 2],
    zorder=5,
    aspect="auto"
)
lead_artist = axs[0].imshow(
    lead_img,
    extent=[0, car_length,
            y_offset - car_height_lead / 2,
            y_offset + car_height_lead / 2],
    zorder=5,
    aspect="auto"
)

# --- Desired gap visualization ---
gap_line, = axs[0].plot([], [], "y--", lw=2, label="Desired Gap", alpha=0.8)
axs[0].legend(loc="upper right")

# --- Info overlay ---
info_text = axs[0].text(0.5, 1.12, '', transform=axs[0].transAxes,
                        ha='center', fontsize=10)

# --- Speed plot ---
axs[1].plot(df["time"], df["lead_v"], "r--", lw=1.5, label="Lead Speed")
axs[1].plot(df["time"], df["ego_v"], "b-", lw=1.5, label="Ego Speed")
axs[1].set_xlim(0, df["time"].max())
axs[1].set_ylim(0, max(df["lead_v"].max(), df["ego_v"].max()) * 1.3)
axs[1].set_ylabel("Speed [m/s]")
axs[1].legend(loc="upper right")

# --- Gap plot ---
axs_gap = axs[1].twinx()
axs_gap.plot(df["time"], df["dist"], "k-", alpha=0.3, lw=1, label="Actual Gap")
axs_gap.set_ylabel("Gap Distance [m]", color="gray")

# --- Throttle & brake plot ---
axs[2].set_xlim(0, df["time"].max())
axs[2].set_ylim(0, 1.05)
axs[2].set_xlabel("Time [s]")
axs[2].set_ylabel("Throttle / Brake")
th_line, = axs[2].plot([], [], "g-", lw=2, label="Throttle")
br_line, = axs[2].plot([], [], "r-", lw=2, label="Brake")
axs[2].legend(loc="upper right")

# --- Mode colors ---
mode_colors = {"Cruise": "#d0f0ff", "Headway": "#fff3b3", "Override": "#ffd6cc"}

# --- Animation update function ---
def update(frame):
    ego_x = df["ego_x"].iloc[frame]
    lead_x = df["lead_x"].iloc[frame]
    dist = df["dist"].iloc[frame]
    time = df["time"].iloc[frame]
    v_ego = df["ego_v"].iloc[frame]
    v_lead = df["lead_v"].iloc[frame]
    mode = df["ACC_mode"].iloc[frame]

    # Update car positions
    ego_artist.set_extent([
        ego_x, ego_x + car_length,
        y_offset - car_height_ego / 2,
        y_offset + car_height_ego / 2
    ])
    lead_artist.set_extent([
        lead_x, lead_x + car_length,
        y_offset - car_height_lead / 2,
        y_offset + car_height_lead / 2
    ])

    # Update background based on ACC mode
    axs[0].set_facecolor(mode_colors.get(mode, "white"))

    # Compute desired spacing
    headway = df["headway"].iloc[frame] if "headway" in df.columns else 1.2
    d0 = df["d_safety"].iloc[frame] if "d_safety" in df.columns else 5.0
    d_des = d0 + headway * v_ego

    # Update desired gap line
    gap_line.set_data([ego_x + car_length, ego_x + car_length + d_des], [y_offset, y_offset])

    # Info text
    info_text.set_text(
        f"t={time:4.1f}s | Mode={mode:8s} | Gap={dist:5.1f} m | "
        f"Desired={d_des:5.1f} m | Ego={v_ego:5.1f} m/s | Lead={v_lead:5.1f} m/s"
    )

    # Update throttle/brake time series
    th_line.set_data(df["time"][:frame], df["throttle"][:frame])
    br_line.set_data(df["time"][:frame], df["brake"][:frame])

    return ego_artist, lead_artist, gap_line, info_text, th_line, br_line

# --- Run animation ---
ani = animation.FuncAnimation(fig, update, frames=len(df), interval=40, blit=True, repeat=False)

# --- Save output ---
ani.save(VIDEO_PATH, fps=25, dpi=150)
print(f"✅ Animation with realistic car images saved to: {VIDEO_PATH}")

plt.show()

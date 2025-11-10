# 🚗 Adaptive Cruise Control (ACC) Simulator

A hybrid **C++ + Python + Gradio** project that simulates and visualizes an **Adaptive Cruise Control (ACC)** system.  
It combines a physics-based C++ backend with an interactive Python visualization — complete with realistic car images, adjustable PID tuning, and simulation playback.

---


## ☁️Hugging Face Space

![ACC Simulation APP](https://huggingface.co/spaces/mayur-waghchoure/acc-pid-simulator)

![ACC Simulation Demo](.\assets\acc_animation_enhanced.mp4)


## 🧠 Key Features

- ⚙️ **C++ Core Simulation** — physics-based ego/lead vehicle dynamics  
- 🧩 **Dual PID Architecture** — cruise (speed) and headway (distance) control  
- 🎨 **Realistic Visualization** — animated ego & lead cars with PNG assets  
- 🌐 **Interactive Gradio App** — tune parameters and view results instantly  
- 🧪 Adjustable parameters for PID gains, initial speeds, gaps, and friction  

---

## 📦 Clone and Setup

### 1️⃣ Clone the Repository

```bash
git clone https://github.com/Mayakshanesht/adaptive_cruise_control_pid.git
cd adaptive_cruise_control_pid
````

---

### 2️⃣ Create and Activate a Virtual Environment

#### On Linux/macOS:

```bash
python3 -m venv venv
source venv/bin/activate
```

#### On Windows (PowerShell):

```bash
python -m venv venv
venv\Scripts\activate
```

---

### 3️⃣ Install Python Dependencies

Make sure you’re inside the project root, then run:

```bash
pip install -r requirements.txt
```

✅ This installs:

* `gradio`
* `matplotlib`
* `pandas`
* `numpy`

---

## ⚙️ Build the C++ Simulator

### 4️⃣ Build the C++ Executable

```bash
mkdir -p build
cd build
cmake ..
make
```

✅ Output:

```
build/acc_simulator
```

---

## 🚀 Run the Simulation (Command Line)

```bash
cd build
./acc_simulator v_set headway mu sim_time init_gap lead_accel v_ego0 v_lead0 Kp_c Ki_c Kd_c Kp_h Ki_h Kd_h
```

#### Example:

```bash
./acc_simulator 27 1.2 0.9 20 40 0 20 20 0.4 0.05 0.01 0.6 0.08 0.02
```

✅ Generates simulation log:

```
build/acc_sim.csv
```

| Parameter              | Description               | Example         |
| ---------------------- | ------------------------- | --------------- |
| `v_set`                | Target cruise speed [m/s] | 20              |
| `headway`              | Time headway [s]          | 1.2             |
| `mu`                   | Road friction coefficient | 0.9             |
| `sim_time`             | Duration [s]              | 20              |
| `init_gap`             | Initial distance [m]      | 30              |
| `lead_accel`           | Lead acceleration [m/s²]  | -0.5              |
| `v_ego0`, `v_lead0`    | Initial speeds [m/s]      | 15, 15          |
| `Kp_c`, `Ki_c`, `Kd_c` | Cruise PID gains          | 0.4, 0.05, 0.01 |
| `Kp_h`, `Ki_h`, `Kd_h` | Headway PID gains         | 0.6, 0.08, 0.02 |

---

## 🎬 Visualize Simulation Results

```bash
cd scripts
python3 visualization.py
```

✅ Loads: `../build/acc_sim.csv`
🎨 Saves: `../build/acc_animation_enhanced.mp4`

**Color Codes:**

* 🟦 Blue — Cruise mode
* 🟨 Yellow — Headway mode
* 🟥 Red — Override (emergency braking)

---

## 🌐 Run the Interactive Gradio App

```bash
cd scripts
python3 app.py
```

Then open the local Gradio interface:

```
http://127.0.0.1:7860
```

You can:

* Adjust PID gains and simulation parameters
* Run simulations interactively
* View results directly in the browser
* Download `.csv` and `.mp4` files

---

## 🧪 PID Tuning Strategy

Ziegler–Nichols tuning isn’t ideal for nonlinear ACC systems.
Use **step-response and lead-follow tests** instead:

| Test             | Goal                     | Adjust                 |
| ---------------- | ------------------------ | ---------------------- |
| Step in `v_set`  | Smooth speed tracking    | `Kp_c`, `Ki_c`, `Kd_c` |
| Lead slows down  | Stable spacing           | `Kp_h`, `Ki_h`, `Kd_h` |
| Emergency brake  | Override behavior        | verify override        |
| Low μ (wet road) | Friction-limited braking | ensure safe decel      |

💡 Ideal tuning = smooth transitions, steady headway, no oscillations.


---

## 📸 Example Output

![ACC Simulation Demo](.\assets\acc_animation_enhanced.mp4)


---


## 👨‍💻 Author

**Mayur Waghchoure**
Autonomous Systems Bootcamp — 2025


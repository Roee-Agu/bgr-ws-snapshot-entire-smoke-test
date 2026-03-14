# Run First Smoke Test

---

## 🟦 Terminal 1 — Start Gazebo

```bash
source /opt/ros/jazzy/setup.bash
source ~/bgr_ws/install/setup.bash
env -u WAYLAND_DISPLAY \
  LIBGL_DRI3_DISABLE=1 \
  QT_QPA_PLATFORM=xcb \
  ros2 launch bgr_description gazebo.launch.py
```

### 🔎 Check (run in another terminal) — expect frequency above **35 Hz**

```bash
ros2 topic list | egrep '(/clock|/model/bgr/odometry|/robot/datalogger_gt|/robot/datalogger_noisy|/robot/mcu_gt|/robot/mcu_noisy)'
ros2 topic hz /model/bgr/odometry
```

---

## 🟩 Terminal 2 — Activate the BGR controllers

```bash
source /opt/ros/jazzy/setup.bash
source ~/bgr_ws/install/setup.bash
ros2 launch bgr_controller controller.launch.py
```

---

## 🟨 Terminal 3 — Reset car position

```bash
source /opt/ros/jazzy/setup.bash
source ~/bgr_ws/install/setup.bash
ros2 run autonomous_car_sim reset_position
```

### ⚠ Important before Terminal 4

make sure to spawn track **'Competition1'**

---

## 🟧 Terminal 4 — EKF

```bash
source /opt/ros/jazzy/setup.bash
cd ~/bgr_ws
source install/setup.bash
ros2 run ekf_project run_simulation --ros-args -p use_sim_time:=true
```

### 🔎 Check (run in another terminal)

Expect:

* **use_sim_time** should be **true**
* **/robot/datalogger_noisy** nominal about **50 Hz**
* **/robot/mcu_noisy** nominal about **50 Hz**
* **/robot/estimated_odom** should be close to **/robot/datalogger_noisy**

```bash
source /opt/ros/jazzy/setup.bash
source ~/bgr_ws/install/setup.bash
ros2 param get /ekf_simulation_runner use_sim_time
ros2 topic hz /robot/datalogger_noisy
ros2 topic hz /robot/mcu_noisy
ros2 topic hz /robot/estimated_odom
```

### ⏳ WAIT FOR

`Startup biases applied: ...`

---

## 🟪 Terminal 5 — Plotter

```bash
source /opt/ros/jazzy/setup.bash
source ~/bgr_ws/install/setup.bash
env -u WAYLAND_DISPLAY QT_QPA_PLATFORM=xcb \
  ros2 run ekf_project live_rmse_plotter --ros-args -p use_sim_time:=true
```

---

## 🟥 Terminal 6 — start path tracker only after startup bias message

```bash
source /opt/ros/jazzy/setup.bash
source ~/bgr_ws/install/setup.bash
ros2 launch autonomous_car_sim autonomous_car.launch.py
```


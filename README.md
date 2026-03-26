# Guide-to-add-new-model-to-PX4-SITL
This is a step by step procedure to add a new model to PX4 SITL
# Adding a New Vehicle to PX4 Gazebo Simulation

https://github.com/username/Subhaaaash/Guide-to-add-new-model-to-PX4-SITL/main/blob/assets/px4_sitl_demo.mp4
A complete step-by-step guide for integrating a custom multirotor vehicle into the PX4 `gz` (Gazebo Garden/Harmonic) simulation environment.

You can also refer to the PDF file in the repository.


---

## Table of Contents

1. [Prerequisites](#1-prerequisites)
2. [Step 1 — Export CAD to DAE Mesh](#2-step-1--export-cad-to-dae-mesh)
3. [Step 2 — Create the Model Directory](#3-step-2--create-the-model-directory)
4. [Step 3 — Write model.config](#4-step-3--write-modelconfig)
5. [Step 4 — Write model.sdf](#5-step-4--write-modelsdf)
6. [Step 5 — Create the PX4 Airframe Script](#6-step-5--create-the-px4-airframe-script)
7. [Step 6 — Register the Airframe in CMakeLists](#7-step-6--register-the-airframe-in-cmakeslists)
8. [Step 7 — Build and Run the Simulation](#8-step-7--build-and-run-the-simulation)
9. [Step 8 — Verify and Tune](#9-step-8--verify-and-tune)
10. [Common Errors and Fixes](#10-common-errors-and-fixes)
11. [File Checklist](#11-file-checklist)
12. [Reference: Key Parameters](#12-reference-key-parameters)

---

## 1. Prerequisites

- Ubuntu 22.04 LTS (recommended)
- PX4 development environment set up:
  ```bash
  bash Tools/setup/ubuntu.sh
  ```
- Gazebo Garden or Harmonic installed (handled by the setup script)
- Submodules initialised:
  ```bash
  git submodule update --init --recursive
  ```
- Verify `Tools/simulation/gz/models/` exists and contains model directories (e.g. `x500/`)

> **Reference vehicle:** Throughout this guide `x500` / `gz_x500` is used as the working reference. It is located at `Tools/simulation/gz/models/x500_base/`.

---

## 2. Step 1 — Export CAD to DAE Mesh

Gazebo uses **Collada (`.dae`)** mesh files for visual geometry and optionally for collision geometry.

### From FreeCAD

1. Open your part/assembly file.
2. Select the body in the model tree.
3. **File → Export → Collada (.dae)**
4. Settings: Unit = millimetre, Tessellation = Fine.
5. Click **OK**.

### From Fusion 360 (via Blender)

Fusion 360 has no native DAE export. Use this workflow:

1. Right-click component → **Save As Mesh → OBJ (with materials)**.
2. Open the OBJ in **Blender** (File → Import → Wavefront OBJ).
3. In Blender: **File → Export → Collada (.dae)**.

### Clean-up in Blender (Recommended)

| Action | How |
|--------|-----|
| Set origin to centre of mass | Object → Set Origin → Origin to Geometry |
| Apply all transforms | Object → Apply → All Transforms |
| Set units to metres | Scene Properties → Units → Metric, Scale = 1.0 |
| Recalculate normals | Edit Mode → A → Mesh → Normals → Recalculate Outside |
| Simplify collision mesh | Add Decimate modifier, Ratio ≈ 0.10–0.20 |

### Coordinate Convention

Gazebo uses **right-hand ENU**: X = forward, Y = left, Z = up. Blender defaults match this.

> **Tip:** Export two meshes — one high-detail for visuals (`frame_visual.dae`) and one low-polygon for collision (`frame_collision.dae`). For small quads, a simple box collision in the SDF is sufficient and avoids the collision mesh entirely.

---

## 3. Step 2 — Create the Model Directory

```
Tools/simulation/gz/models/
  my_vehicle/
    model.config        ← metadata
    model.sdf           ← physics, sensors, plugins
    meshes/
      frame_visual.dae
      propeller_cw.dae   (optional)
      propeller_ccw.dae  (optional)
```

```bash
cd ~/PX4-Autopilot/Tools/simulation/gz/models
mkdir -p my_vehicle/meshes
cp /path/to/frame_visual.dae   my_vehicle/meshes/
cp /path/to/propeller_cw.dae   my_vehicle/meshes/
cp /path/to/propeller_ccw.dae  my_vehicle/meshes/
```

> **Important:** The directory name must exactly match the value you will set for `PX4_SIM_MODEL` in the airframe script.

---

## 4. Step 3 — Write model.config

Create `Tools/simulation/gz/models/my_vehicle/model.config`:

```xml
<?xml version="1.0"?>
<model>
  <name>my_vehicle</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>you@example.com</email>
  </author>
  <description>Custom quadrotor vehicle for PX4 simulation.</description>
</model>
```

| Field | Rule |
|-------|------|
| `<name>` | Must match directory name and `PX4_SIM_MODEL` exactly |
| `<sdf version>` | Use `1.9` for Gazebo Garden/Harmonic |
| `<sdf>` content | Filename of the SDF file (usually `model.sdf`) |

---

## 5. Step 4 — Write model.sdf

The SDF file defines rigid bodies, joints, sensors, and motor plugins.

### 5.1 File Skeleton

```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="my_vehicle">
    <pose>0 0 0.24 0 0 0</pose>   <!-- spawn height above ground -->

    <!-- (A) Base link: inertia + visual + collision + sensors -->
    <link name="base_link"> ... </link>

    <!-- (B) Rotor links and joints (one per motor) -->
    <link name="rotor_0"> ... </link>
    <joint name="rotor_0_joint" type="revolute"> ... </joint>
    <!-- repeat for rotor_1, rotor_2, rotor_3 -->

    <!-- (C) Motor model plugins (one per motor) -->
    <plugin filename="gz-sim-multicopter-motor-model-system"
            name="gz::sim::systems::MulticopterMotorModel">
      <!-- motor 0 config -->
    </plugin>
    <!-- repeat for motors 1, 2, 3 -->
  </model>
</sdf>
```

### 5.2 Base Link — Inertia and Mesh

```xml
<link name="base_link">
  <gravity>true</gravity>
  <inertial>
    <mass>1.5</mass>                     <!-- kg; from CAD mass properties -->
    <inertia>                             <!-- kg·m²; from CAD mass properties -->
      <ixx>0.029125</ixx> <ixy>0</ixy> <ixz>0</ixz>
      <iyy>0.029125</iyy> <iyz>0</iyz>
      <izz>0.055225</izz>
    </inertia>
  </inertial>

  <visual name="base_link_visual">
    <geometry><mesh>
      <uri>model://my_vehicle/meshes/frame_visual.dae</uri>
    </mesh></geometry>
  </visual>

  <collision name="base_link_collision">
    <geometry><box>
      <size>0.35 0.35 0.05</size>        <!-- replace with actual bounding box -->
    </box></geometry>
  </collision>

  <!-- sensors go here (see 5.3) -->
</link>
```

### 5.3 Sensors (inside base_link)

Add the following four sensor blocks. These are the **minimum required sensors** for PX4 SITL.

```xml
<!-- IMU: 250 Hz -->
<sensor name="imu_sensor" type="imu">
  <gz_frame_id>base_link</gz_frame_id>
  <always_on>1</always_on>
  <update_rate>250</update_rate>
  <imu>
    <angular_velocity>
      <x><noise type="gaussian"><mean>0</mean><stddev>0.0008726646</stddev></noise></x>
      <y><noise type="gaussian"><mean>0</mean><stddev>0.0008726646</stddev></noise></y>
      <z><noise type="gaussian"><mean>0</mean><stddev>0.0008726646</stddev></noise></z>
    </angular_velocity>
    <linear_acceleration>
      <x><noise type="gaussian"><mean>0</mean><stddev>0.00637</stddev></noise></x>
      <y><noise type="gaussian"><mean>0</mean><stddev>0.00637</stddev></noise></y>
      <z><noise type="gaussian"><mean>0</mean><stddev>0.00686</stddev></noise></z>
    </linear_acceleration>
  </imu>
</sensor>

<!-- Barometer: 50 Hz -->
<sensor name="air_pressure_sensor" type="air_pressure">
  <gz_frame_id>base_link</gz_frame_id>
  <always_on>1</always_on>
  <update_rate>50</update_rate>
  <air_pressure><pressure>
    <noise type="gaussian"><mean>0</mean><stddev>3</stddev></noise>
  </pressure></air_pressure>
</sensor>

<!-- Magnetometer: 100 Hz -->
<sensor name="magnetometer_sensor" type="magnetometer">
  <gz_frame_id>base_link</gz_frame_id>
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <magnetometer>
    <x><noise type="gaussian"><stddev>0.0001</stddev></noise></x>
    <y><noise type="gaussian"><stddev>0.0001</stddev></noise></y>
    <z><noise type="gaussian"><stddev>0.0001</stddev></noise></z>
  </magnetometer>
</sensor>

<!-- GPS: 30 Hz -->
<sensor name="navsat_sensor" type="navsat">
  <gz_frame_id>base_link</gz_frame_id>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
</sensor>
```

> ⚠️ **CRITICAL — Do NOT add sensor system plugins to model.sdf**
>
> Do **not** add any of these plugin tags inside your model SDF:
> - `gz-sim-imu-system`
> - `gz-sim-magnetometer-system`
> - `gz-sim-air-pressure-system`
> - `gz-sim-navsat-system`
>
> These are already loaded **globally** by `Tools/simulation/gz/server.config`.
> Adding them again at the model level causes every sensor to fire **twice** per
> simulation step, producing duplicate IMU messages with identical timestamps.
> PX4's `vehicle_imu` module detects this as a timestamp error and the simulation
> crashes with:
> ```
> ERROR [vehicle_imu] 0 - gyro XXXXXX timestamp error timestamp_sample: ...
> ```
> The `x500_base` reference model has **no** sensor system plugin tags — only
> `<sensor>` definitions. Follow that pattern.

### 5.4 Rotor Links and Joints

Standard X-quad motor numbering (PX4 convention):

| Motor | Position | Spin | KM sign |
|-------|----------|------|---------|
| 0 | Front-Right | CCW | Negative (`-0.05`) |
| 1 | Rear-Left | CCW | Negative (`-0.05`) |
| 2 | Front-Left | CW | Positive (`+0.05`) |
| 3 | Rear-Right | CW | Positive (`+0.05`) |

```xml
<!-- Motor 0: Front-Right, CCW -->
<link name="rotor_0">
  <gravity>true</gravity>
  <pose>0.13 -0.22 0.023 0 0 0</pose>  <!-- XYZ from CoM in metres -->
  <inertial>
    <mass>0.016</mass>
    <inertia><ixx>3.8e-07</ixx><iyy>2.6e-05</iyy><izz>2.6e-05</izz></inertia>
  </inertial>
  <visual name="rotor_0_visual">
    <geometry><cylinder>
      <radius>0.12</radius><length>0.005</length>
    </cylinder></geometry>
  </visual>
  <collision name="rotor_0_collision">
    <geometry><box><size>0.24 0.017 0.001</size></box></geometry>
  </collision>
</link>

<joint name="rotor_0_joint" type="revolute">
  <parent>base_link</parent>
  <child>rotor_0</child>
  <axis>
    <xyz>0 0 1</xyz>
    <limit><lower>-1e+16</lower><upper>1e+16</upper></limit>
    <dynamics><spring_reference>0</spring_reference><spring_stiffness>0</spring_stiffness></dynamics>
  </axis>
</joint>
<!-- Repeat for rotor_1 (ccw), rotor_2 (cw), rotor_3 (cw) -->
```

### 5.5 Motor Model Plugins

```xml
<!-- Motor 0: Front-Right, CCW -->
<plugin filename="gz-sim-multicopter-motor-model-system"
        name="gz::sim::systems::MulticopterMotorModel">
  <jointName>rotor_0_joint</jointName>
  <linkName>rotor_0</linkName>
  <turningDirection>ccw</turningDirection>
  <timeConstantUp>0.0125</timeConstantUp>
  <timeConstantDown>0.025</timeConstantDown>
  <maxRotVelocity>1000.0</maxRotVelocity>
  <motorConstant>8.54858e-06</motorConstant>   <!-- thrust coefficient -->
  <momentConstant>0.016</momentConstant>        <!-- torque/thrust ratio -->
  <commandSubTopic>command/motor_speed</commandSubTopic>
  <motorNumber>0</motorNumber>
  <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
  <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
  <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
  <motorType>velocity</motorType>
</plugin>
<!-- Repeat for motors 1 (ccw), 2 (cw), 3 (cw) -->
```

---

## 6. Step 5 — Create the PX4 Airframe Script

Create a new file in `ROMFS/px4fmu_common/init.d-posix/airframes/`.

**Naming:** `XXXX_gz_my_vehicle` where `XXXX` is a unique 4-digit number.
- PX4 upstream uses `4001–4099` for gz models.
- Use `22000–22999` for custom/personal vehicles (reserved range).

```sh
#!/bin/sh
#
# @name Gazebo my_vehicle
#
# @type Quadrotor
#

. ${R}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=my_vehicle}   # must match model directory name

param set-default SIM_GZ_EN 1

# ── Control Allocation ──────────────────────────────────────────────────────
param set-default CA_AIRFRAME 0      # 0 = multirotor
param set-default CA_ROTOR_COUNT 4

# Motor positions (metres from CoM) and yaw torque sign
# KM < 0 = CCW spin (negative yaw torque)
# KM > 0 = CW  spin (positive yaw torque)

param set-default CA_ROTOR0_PX  0.13   # Front-Right  CCW
param set-default CA_ROTOR0_PY -0.22
param set-default CA_ROTOR0_KM -0.05

param set-default CA_ROTOR1_PX -0.13   # Rear-Left    CCW
param set-default CA_ROTOR1_PY  0.22
param set-default CA_ROTOR1_KM -0.05

param set-default CA_ROTOR2_PX  0.13   # Front-Left   CW
param set-default CA_ROTOR2_PY  0.22
param set-default CA_ROTOR2_KM  0.05

param set-default CA_ROTOR3_PX -0.13   # Rear-Right   CW
param set-default CA_ROTOR3_PY -0.22
param set-default CA_ROTOR3_KM  0.05

# ── ESC Output Mapping ──────────────────────────────────────────────────────
param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_EC_FUNC3 103
param set-default SIM_GZ_EC_FUNC4 104

param set-default SIM_GZ_EC_MIN1 150
param set-default SIM_GZ_EC_MIN2 150
param set-default SIM_GZ_EC_MIN3 150
param set-default SIM_GZ_EC_MIN4 150

param set-default SIM_GZ_EC_MAX1 1000
param set-default SIM_GZ_EC_MAX2 1000
param set-default SIM_GZ_EC_MAX3 1000
param set-default SIM_GZ_EC_MAX4 1000

# ── Flight Controller Defaults ──────────────────────────────────────────────
param set-default MPC_THR_HOVER 0.60   # hover throttle (tune after first flight)
param set-default NAV_DLL_ACT 2
```

---

## 7. Step 6 — Register the Airframe in CMakeLists

Open `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` and add your airframe filename to the `px4_add_romfs_files()` list:

```cmake
px4_add_romfs_files(
    ...
    4001_gz_x500
    4002_gz_x500_depth
    ...
    4099_gz_my_vehicle   # ← ADD THIS
    ...
)
```

> The `gz_bridge` CMakeLists automatically creates a make target `gz_<model_name>` for every airframe file matching the pattern `*_gz_*`. Once registered and rebuilt, `make px4_sitl gz_my_vehicle` will be available.

---

## 8. Step 7 — Build and Run the Simulation

### First Build (or after changing ROMFS files)

```bash
cd ~/PX4-Autopilot
make distclean
make px4_sitl gz_my_vehicle
```

### Incremental Build

```bash
make px4_sitl gz_my_vehicle
```

### With a Different World

```bash
PX4_GZ_WORLD=windy make px4_sitl gz_my_vehicle
```

### Headless Mode (no GUI)

```bash
HEADLESS=1 make px4_sitl gz_my_vehicle
```

### Successful Launch Output

```
INFO  [px4] Startup script returned successfully
INFO  [gz_bridge] model: my_vehicle
INFO  [gz_bridge] world: default
INFO  [gz_bridge] Subscribed to /world/default/model/my_vehicle/...
INFO  [commander] Ready for takeoff!
pxh>
```

---

## 9. Step 8 — Verify and Tune

### Check Sensor Data (pxh console)

```bash
listener sensor_accel          # IMU accelerometer
listener sensor_gyro           # IMU gyroscope
listener sensor_baro           # barometer
listener vehicle_gps_position  # GPS fix
ekf2 status                    # EKF2 health
control_allocator status       # control allocation
```

### First Takeoff Test

```bash
commander arm
commander takeoff
```

### Tune Hover Throttle

If the vehicle climbs with stick centred, lower `MPC_THR_HOVER`.
If it descends, raise it.

```bash
param set MPC_THR_HOVER 0.55
```

### PID Tuning

```bash
param set MC_ROLLRATE_P  0.15
param set MC_PITCHRATE_P 0.15
param set MC_YAWRATE_P   0.2
```

---

## 10. Common Errors and Fixes

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `ERROR [vehicle_imu] timestamp error` | Sensor system plugins duplicated (model + server.config) | Remove `gz-sim-imu-system`, `gz-sim-air-pressure-system`, `gz-sim-magnetometer-system`, `gz-sim-navsat-system` from `model.sdf` |
| Vehicle immediately flips on takeoff | Motor spin direction mismatch | Verify `turningDirection` in SDF matches `CA_ROTORx_KM` sign in airframe |
| Vehicle drifts in one axis | Incorrect motor arm positions | Measure actual X/Y from CoM and update `CA_ROTORx_PX/PY` |
| `gz_my_vehicle` make target missing | Airframe not in CMakeLists | Add filename to `airframes/CMakeLists.txt` and run `make distclean` then rebuild |
| Model not found / Gazebo error | Name mismatch across 3 places | Ensure directory name = `model.config <name>` = `PX4_SIM_MODEL` |
| Mesh not visible in Gazebo | Wrong URI path | Use `model://my_vehicle/meshes/file.dae` format |
| Vehicle falls through ground | Spawn height too low or no collision | Check `<pose>` in `model.sdf`; add collision geometry to `base_link` |

---

## 11. File Checklist

Before running the simulation, verify all these files exist:

- [ ] `Tools/simulation/gz/models/my_vehicle/model.config`
- [ ] `Tools/simulation/gz/models/my_vehicle/model.sdf`
- [ ] `Tools/simulation/gz/models/my_vehicle/meshes/frame_visual.dae` *(if using mesh)*
- [ ] `ROMFS/px4fmu_common/init.d-posix/airframes/XXXX_gz_my_vehicle`
- [ ] `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` *(entry added)*

---

## 12. Reference: Key Parameters

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `CA_AIRFRAME` | Airframe type (0=MC) | `0` |
| `CA_ROTOR_COUNT` | Number of motors | `4` |
| `CA_ROTORx_PX/PY` | Rotor position from CoM (m) | `±0.13 / ±0.22` |
| `CA_ROTORx_KM` | Yaw torque sign (±0.05) | `±0.05` |
| `SIM_GZ_EN` | Enable Gazebo bridge | `1` |
| `SIM_GZ_EC_FUNCn` | ESC function mapping | `101–104` |
| `SIM_GZ_EC_MINn/MAXn` | ESC PWM range | `150 / 1000` |
| `MPC_THR_HOVER` | Hover throttle (0–1) | `0.50–0.65` |
| `MPC_THR_MIN` | Minimum thrust | `0.12` |
| `MC_ROLLRATE_P` | Roll rate P gain | `0.15` |
| `MC_PITCHRATE_P` | Pitch rate P gain | `0.15` |

---

## Directory Structure Summary

```
~/PX4-Autopilot/
├── ROMFS/px4fmu_common/init.d-posix/airframes/
│   ├── CMakeLists.txt                   ← add airframe filename here
│   └── XXXX_gz_my_vehicle               ← airframe script
│
└── Tools/simulation/gz/
    ├── server.config                    ← global plugins (DO NOT modify)
    ├── worlds/default.sdf               ← world file
    └── models/
        └── my_vehicle/                  ← directory name = PX4_SIM_MODEL
            ├── model.config
            ├── model.sdf
            └── meshes/
                ├── frame_visual.dae
                ├── propeller_cw.dae
                └── propeller_ccw.dae
```

---

*PX4 Autopilot — docs.px4.io*

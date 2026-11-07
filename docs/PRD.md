# PRD: Stanford DroneHacks 2026 Autonomous Flight Stack (Team-08)

## 0. Architectural Rationale & Literature Adaptations (MonoRace / SkyDreamer / VIO)

### 0.1 Why Direct-to-Motor RL Control is Rejected for Sky Beta
While end-to-end neural motor policies ($u_1, u_2, u_3, u_4$) in **MonoRace** and **SkyDreamer** eliminate inner-loop PID lag and actuator saturation during $6g\text{--}7g$ racing maneuvers at $75\text{--}100\text{ km/h}$, mapping motor commands straight from the Jetson on the **Sky Beta** kit is counterproductive for four reasons:
1. **Starter Interface & Rule 8.5 Constraints:** The kit uses `skybeta_flight_interface`, and the organisers' guide states: *"Do not publish to `/fmu/in/*` yourself."* Furthermore, **Rule 8.5** strictly prohibits modifying the organisers' PX4 safety settings, failsafes, or limits. Bypassing PX4's attitude/rate controllers to send raw `ActuatorMotors` commands breaks the `COM_OF_LOSS_T` ($1\text{ s}$ Offboard loss) fallback to Position mode.
2. **Control Loop Rate & Latency Mismatch:** MonoRace runs `G&CNet` at $500\text{ Hz}$ directly on the STM32H743 microcontroller, and SkyDreamer runs at $90\text{ Hz}$ on the Jetson coupled with a $2000\text{ Hz}$ gyro stream and bidirectional DShot motor RPM feedback ($\omega_1..\omega_4$). On the Sky Beta kit, stereo runs at $30\text{ fps}$, the IMU at $200\text{ Hz}$, and commands cross a `uXRCE-DDS` bridge with a **$68.7\text{ ms}$ median latency**. Stabilizing raw motor thrusts over a $\sim 69\text{ ms}$ delayed link without RPM feedback causes immediate oscillation.
3. **The $1.0\text{ m/s}$ Speed Cap:** PX4 on the kit is capped at `MPC_XY_VEL_MAX = 1.0` $\text{m/s}$. At $1.0\text{ m/s}$, aerodynamic drag and non-linear rotor wash are negligible.
4. **How We Bypass PID Lag Legally (50 Hz Led-Pose Feed-Forward):** Standard PID position tracking lags by $\sim 17\text{ s}$ across the DroneHacks course and overshoots at the first maze corner on PX4 v1.16.1's attitude tune. By leading the $50\text{ Hz}$ position setpoint using the trajectory's velocity ($v$) and acceleration ($a$) scaled by PX4's internal P-gains ($k_p = 0.95, k_v = 1.80$), we turn PX4's cascaded PIDs into a zero-lag feed-forward tracker without violating Rule 8.5.

### 0.2 Key Techniques Adapted from MonoRace, SkyDreamer & VIO Standards
1. **Dynamic Covariance ($\mathbf{R}_k$) & Health State Gating (`odom_guard`):** Inspired by MonoRace's measurement noise scaling $\sigma_{\text{pos}}^2 \propto d^2 / (N_c^2 \cdot N_g)$ and VIO health enums (`HEALTHY`, `DEGRADED`, `LOST`), `odom_guard` intercepts `cuVSLAM`'s confident-but-wrong stationary poses on blank walls and inside the dark tunnel, inflates variances during degraded texture/high yaw rates, and withholds invalid frames so PX4's EKF2 dead-reckons cleanly on the IMU and downward rangefinder.
2. **Seamless Re-Anchoring (`reset_counter` Protocol):** Withholding blank frames in the dark tunnel and letting `cuVSLAM` resume raw caused a $2.2\text{ m}$ jump ($149\text{ cm}$ RMS error) in simulation. On recovery (`LOST -> HEALTHY`), `odom_guard` updates a rigid 6-DoF offset transform $\mathbf{T}_{\text{anchor}}$ so the resumed visual track continues seamlessly from PX4's dead-reckoned `/fmu/out/vehicle_local_position`.
3. **Landmark-Based Map Anchoring (2D Depth Slice ICP vs. GateNet PnP):** Instead of U-Net gate segmentation masks, `map_corrector` slices the OAK-D's stereo depth at $0.80\text{ m}$ flight height and uses an Iterated Kalman Filter + ICP to align observed walls and the C2a Window against the preloaded v3 CAD map (cutting drift from $8\text{ cm}$ to $2\text{ cm}$ while staying 100% compliant with Rule 6.4).
4. **Adaptive Frustum Culling:** Inspired by MonoRace's state-based ROI cropping, `map_corrector` uses the current EKF pose to frustum-cull depth points outside the $6.096 \times 6.096\text{ m}$ net boundary so background room objects seen through the mesh net never corrupt wall matching.
5. **Active Perception Look-Ahead Gaze:** Facing a maze end-wall at $0.3\text{ m}$ starves `cuVSLAM` (withholding 390 of 543 frames). `trajectory_follower` points the nose $0.80\text{ m}$ ahead along the path so the camera rotates into the next corridor before reaching the corner.
6. **Dual Competition Profiles (`bank` vs. `attack`):** Inspired by MonoRace's `M23` (88.4% safe qualifier) vs. `M16` (aggressive finals) policies, our launch system supports `--profile=bank` (guarantees a clean leaderboard finish on Attempt 1) and `--profile=attack` (Window $+5\text{ pts}$ + autonomous Return trip $\times 1.25$ multiplier).

---

## 1. Project Overview & Strategic Rules
**Objective:** Build a fully autonomous, GPS-denied onboard software stack for the issued **Sky Beta** quadcopter to navigate an indoor obstacle course (First Corridor $\rightarrow$ Maze $\rightarrow$ Dark Tunnel $\rightarrow$ Chicane/Window $\rightarrow$ Landing Pad $\rightarrow$ Return Flight to Launch) within a 3-minute limit after a single start command.

### Strategic North Star
1. **Finish First, Speed Second:** Every finished run flown below the walls ($1.727\text{ m}$ / $68\text{ in}$) scores strictly higher than every unfinished run. Top speed is capped at $1.0\text{ m/s}$ (`MPC_XY_VEL_MAX = 1.0`), meaning cornering smoothness and trajectory tracking lag—not top speed—determine run time.
2. **Surgical Code Footprint:** Use the kit's provided software (`depthai-ros`, Isaac ROS `cuVSLAM` stereo-only, `px4_odom_bridge.py`, and PX4 v1.16.1 EKF2) wherever it works. Write custom nodes **only** where simulation and hardware benchmarks proved failures:
   - **`odom_guard`**: Intercepts `cuVSLAM` to filter confident-but-wrong blank-wall/dark-tunnel poses, dynamically scales covariance ($\mathbf{R}_k$), and re-anchors coordinates after dropouts without jumps.
   - **`course_planner`**: Generates collision-free paths from the preloaded v3 CAD map (and live wall-offset updates) using A* with Nav2 inflation costs and line-of-sight shortcutting. Zero hard-coded waypoints (protects the 15-pt Generality score and passes Rule 9.5 AI code review).
   - **`trajectory_follower`**: Computes arc-blended, jerk-limited trajectories with $0.8\text{ m}$ active-perception look-ahead yaw and $50\text{ Hz}$ velocity/acceleration **Led Pose** feed-forward compensation.
   - **`map_corrector` (Stretch / Precision Multiplier):** Slices stereo depth at flight height, frustum-culls points outside the net, and runs Iterated Kalman Filter + ICP wall-matching to eliminate VIO drift and estimate physical wall-placement offsets.
3. **Strict Rule Compliance:**
   - **Rule 5.1:** Fully autonomous after ONE start command. Laptop is read-only during runs.
   - **Rule 6.4 (Maps Allowed, Run Data Banned):** Preloading the published v3 CAD map (and tape-measure survey corrections of the physical course) is 100% legal. Saving SLAM maps, point clouds, or Gaussian splats from one flight to feed into a subsequent flight is strictly prohibited. All live wall-offset estimates must reset between runs. Offline parameter tuning (e.g., camera extrinsics, EKF noise, PID lead gains) from flight logs is legal.
   - **Rule 8.5:** NEVER modify the organisers' PX4 safety parameters (`COM_OF_LOSS_T`, `COM_POSCTL_NAVL`, `NAV_RCL_ACT`, `COM_RC_OVERRIDE`, `COM_FAIL_ACT_T`, `COM_KILL_DISARM`, geofences, or battery limits).
   - **Rule 9.5:** AI code review screens the repository for hard-coded coordinates, timings, or routes. All goals (tunnel axis, window center, landing pad) must be parsed programmatically from the map file.

---

## 2. Hardware, Firmware & Environment Specifications

### 2.1 The "Sky Beta" Kit (`SN0001` Reference)
- **Airframe:** Carbon quadcopter, $4\times$ DYS X2807 $1500\text{ KV}$ motors, 7-inch props, 4S $5000\text{ mAh}$ LiPo, mass $\sim 1.25\text{ kg}$.
- **Dimensions:** Footprint $0.37 \times 0.48\text{ m}$; propeller reach $0.28\text{ m}$ radius from center. Resting body height on launch mark: $\sim 0.07\text{ m}$ (verify with tape measure on Day 1).
- **Companion Computer:** NVIDIA Jetson Orin Nano 8 GB (Ubuntu / JetPack 6, ROS 2 Humble).
  - *Note:* The hosted **FlyX Lab** cloud simulator runs ROS 2 **Jazzy**, `cuVSLAM` 17, $640\times 400$ stereo @ $15\text{ Hz}$, and lacks OpenCV. Code must remain compatible with both Humble and Jazzy.
- **Flight Controller:** Holybro Pixhawk 6C running **PX4 v1.16.1**, communicating via `uXRCE-DDS` (`ROS_DOMAIN_ID = 1`, matching `UXRCE_DDS_DOM_ID = 1`).
- **Primary Vision Sensor:** Luxonis **OAK-D S2** mounted forward and level, $\sim 0.11\text{ m}$ ahead of center.
  - Stereo pair: $2\times$ 1 MP global-shutter mono cameras, $75\text{ mm}$ baseline, $1024\times 640$ @ $30\text{ fps}$ via `depthai-ros`.
  - Internal IMU: $200\text{ Hz}$.
- **Height Sensor:** Holybro **VL53L1X** downward Time-of-Flight (ToF) rangefinder (up to $4\text{ m}$ range, $27^\circ$ FOV).

### 2.2 Course Geometry & Coordinate Frames
- **World Map Frame (Course CAD v3):** Origin $(0.0, 0.0)$ at floor center; $+X$ East, $+Y$ North, $+Z$ Up (ENU).
  - **Flight Net:** $6.096 \times 6.096\text{ m}$ ($20\times 20\text{ ft}$), $3.048\text{ m}$ ($10\text{ ft}$) tall. Bounds: $x, y \in [-3.048, +3.048]$.
  - **Walls:** $1.727\text{ m}$ ($68\text{ in}$) tall, $0.102\text{ m}$ ($4\text{ in}$) thick. Drone must keep some part of airframe below $1.727\text{ m}$ at all times (nominal flight altitude: **$0.80\text{ m}$ above floor** $\rightarrow z_{\text{setpoint}} = 0.80 - z_{\text{rest}} \approx 0.73\text{ m}$).
  - **Corridors:** $1.02\text{--}1.07\text{ m}$ wide; narrowest bottom passage is $0.93\text{ m}$ wide. With $0.28\text{ m}$ prop reach, total lateral error budget in the narrowest passage is **$\pm 0.18\text{ m}$**.
  - **Launch Mark:** $(-2.515, +0.914)\text{ m}$, facing **North ($+Y$)** up the first corridor.
  - **Dark Tunnel (Checkpoint 3 - 10 pts):** Oval tube ($58 \times 72 \times 60\text{ in}$, $\sim 1.5\text{ m}$ long) against the East net. Dark interior; lit exit spans $52^\circ$ of the $80^\circ$ FOV from entrance.
  - **Window in Wall C2a (Checkpoint 4 Bonus - +5 pts):** $0.914 \times 0.914\text{ m}$ opening, $0.305\text{ m}$ to $1.219\text{ m}$ above floor. Center height is $0.762\text{ m}$ (matches our $0.80\text{ m}$ flight altitude).
  - **Boulders ($\times 5$):** $0.457\text{ m}$ diameter, $0.533\text{ m}$ tall cylinders in the southern section. *Hazard:* If flown over, the downward VL53L1X rangefinder reads them as a $+0.53\text{ m}$ floor step, causing EKF2 to spike altitude.
  - **Landing Pad (Checkpoint 5 - 7 pts):** $1.524\text{ m}$ ($5\text{ ft}$) outer ring diameter, centered at $(-2.042, -0.792)\text{ m}$.
- **Local Tracker / ROS Frame (ENU / FLU):**
  - Origin $(0, 0, 0)$ is initialized where the drone rests on the Launch Mark facing North.
  - Because the drone starts facing **North ($+Y_{\text{course}}$)**, the local ROS $+X_{\text{local}}$ (Forward) axis aligns with $+Y_{\text{course}}$ (North), and $+Y_{\text{local}}$ (Left) aligns with $-X_{\text{course}}$ (West).
  - **Coordinate Transform (Course Map $(E, N) \rightarrow$ Local ROS $(x_{\text{local}}, y_{\text{local}})$):**
    ```python
    dx_east = E - (-2.515)
    dy_north = N - 0.914
    x_local = dy_north          # North is +X forward
    y_local = -dx_east          # West is +Y left (-90 deg rotation)
    z_local = 0.80 - z_rest_m   # Nominal 0.80m altitude minus resting height (~0.07m)
    ```
- **PX4 Frame (NED / FRD):**
  - Handled by `px4_odom_bridge.py` and `skybeta_flight_interface`. Never publish raw `/fmu/in/*` messages directly from mission nodes. All PX4 timestamps are in **microseconds**.

---

## 3. System Architecture & Node Specifications

```text
[OAK-D S2: Stereo 30Hz] ---> [depthai-ros] ---> [Isaac ROS cuVSLAM (Stereo-Only)]
                                                        |
                                                        v  (/visual_slam/tracking/odometry)
[PX4 VehicleLocalPosition] -------------------> [1. odom_guard] (Gating, Re-anchor, Dynamic R_k)
        |                                               |
        |                                               v  (/guarded/odometry)
        |                    [4. map_corrector] <-------+ (Optional ICP Wall Alignment)
        |                            |
        |                            v
        |                  [px4_odom_bridge.py] ------> [PX4 EKF2 (v1.16.1)]
        |                                                       ^
        |                                                       |
        |                                           [VL53L1X Rangefinder (HGT_REF)]
        v
[Course v3 CAD Map] ---> [2. course_planner] ---> [3. trajectory_follower] ---> [Patched Flight Interface] ---> [PX4 Offboard]
```

### 3.1 Patch 0: `skybeta_flight_interface` Two-Line Fix & Rate Upgrade
- **Problem Measured in Sim:** The starter `skybeta_flight_interface` continuously publishes a startup hold-pose setpoint in its `tick()` timer even before Offboard mode is active. This fights PX4's internal takeoff/mode logic and causes the drone to yaw wildly (up to $100^\circ$ error) on takeoff. Furthermore, its default setpoint rate is too low for tight cornering.
- **Required Change:**
  1. In `tick()`, always publish the `OffboardControlMode` heartbeat (required at $>2\text{ Hz}$), but **gate `TrajectorySetpoint` publishing** so setpoints are only sent once `nav_state == NAVIGATION_STATE_OFFBOARD` (or during the explicit pre-switch priming window).
  2. Increase `setpoint_rate_hz` parameter to **`50.0` Hz**.

### 3.2 PX4 Parameter & Pre-Flight Calibration Protocol
- **Backup First:** Dump all parameters via QGroundControl (`SN0001_backup.params`) before changing anything.
- **Allowed Estimator Tweaks (Do NOT touch safety/failsafe params):**
  - `EKF2_EVA_NOISE = 0.01` (PX4 v1.16.1 bug uses this parameter unsquared, making default vision yaw weight $5\times$ weaker than v1.18).
  - `EKF2_EV_QMIN = 0` (Starter bridge emits `quality = 0`; any `QMIN > 0` rejects 100% of vision messages).
  - `EKF2_EV_DELAY = 0` (Starter bridge already timestamps messages at image capture time; leaving default $70\text{ ms}$ double-counts latency and causes wall strikes in 4 of 5 runs).
- **Camera Exposure & Timestamping (`depthai-ros`):**
  - Cap auto-exposure maximum shutter time to **$8\text{--}10\text{ ms}$** and meter on the central ROI so the dark tunnel walls do not over-expose the lit exit and motion blur stays $4\times$ lower.
  - Enable `i_add_exposure_offset: true` to timestamp frames at mid-exposure.
- **Mandatory 30-Second Settle & Extrinsic / Heading Gate:**
  - Before arming, keep the drone completely stationary on the Launch Mark with `cuVSLAM` and `odom_guard` running for **30 seconds**.
  - Estimate any static camera pitch/roll mount offset ($\theta_{\text{cam}}$) during settle.
  - Block mission start until:
    1. `estimator_sensor_bias.gyro_bias[2]` (yaw gyro bias) is $< 0.001\text{ rad/s}$ and steady.
    2. $|\psi_{\text{px4}} - \psi_{\text{vision}}| < 2.0^\circ$.

---

### 3.3 Node 1: `odom_guard` (Vision Fault Protection & Re-Anchoring)
**Purpose:** `cuVSLAM` fails silently on textureless maze walls and inside the dark tunnel—reporting hundreds of features and a stationary pose with tiny covariance while the drone is actually moving. Passing this to PX4 causes runaway acceleration into walls.

**Inputs:**
- `/visual_slam/tracking/odometry` (`nav_msgs/Odometry` from `cuVSLAM`)
- `/oak/left/image_rect_raw` (or stereo image for fast texture/gradient check)
- `/fmu/out/vehicle_local_position` & `/fmu/out/vehicle_attitude` (Best-Effort QoS)
- `/fmu/out/sensor_combined` (for gyroscope angular velocity $\Omega_{\text{imu}}$)

**Outputs:**
- `/guarded/odometry` (`nav_msgs/Odometry` fed into `px4_odom_bridge.py`)
- `/guard/health_state` (`uint8`: `0 = HEALTHY`, `1 = DEGRADED`, `2 = LOST`)

**Core Logic & Mathematical Gating:**
1. **Frame Validity Checks (Withhold Frame & Declare `LOST` if ANY trigger):**
   - **Texture Starvation:** Compute fast image gradient / edge density (e.g., Sobel or Laplacian variance on downsampled stride-4 image). If textured pixel ratio $< 1.0\%$, withhold frame.
   - **Velocity Spike:** If implied linear speed $\|\Delta \mathbf{p} / \Delta t\| > 10.0\text{ m/s}$ or angular speed $\|\Delta \mathbf{R} / \Delta t\| > 720^\circ\text{/s}$ ($12.56\text{ rad/s}$), withhold frame.
   - **Gyro Disagreement (MonoRace Innovation Gate):** Compare frame-to-frame visual rotation rate $\Omega_{\text{vis}}$ against low-pass filtered IMU gyro rate $\Omega_{\text{imu}}$:
     $$\|\Omega_{\text{vis}} - \Omega_{\text{imu}}\|_2 > \tau_{\text{gyro}} \quad (\text{default } \tau_{\text{gyro}} = 0.8\text{ rad/s})$$
   - **Mahalanobis Position Gate:** Reject if visual innovation against PX4's current prediction exceeds $4\sigma$ of the expected uncertainty envelope.
2. **State Machine & Seamless Re-Anchoring (`reset_counter` logic):**
   - Maintain a rigid 6-DoF anchor transform $\mathbf{T}_{\text{anchor}} = (\mathbf{R}_{\text{anchor}}, \mathbf{t}_{\text{anchor}})$ mapping raw `cuVSLAM` poses into the continuous guarded frame:
     $$\mathbf{T}_{\text{guarded}}(t) = \mathbf{T}_{\text{anchor}} \cdot \mathbf{T}_{\text{cuvslam}}(t)$$
   - While `LOST`: Stop publishing to `/guarded/odometry`. PX4 EKF2 automatically coasts on IMU + VL53L1X rangefinder (survives the $1.5\text{ m}$ dark tunnel cleanly). If `LOST` persists for $> 30\text{ consecutive frames}$ ($1.0\text{ s}$), trigger a `cuVSLAM` reset service call.
   - On transition `LOST -> HEALTHY` (requires $K = 3$ consecutive valid frames): Recompute $\mathbf{T}_{\text{anchor}}$ so the newly recovered visual pose seamlessly equals PX4's current dead-reckoned pose (`vehicle_local_position` / `vehicle_attitude`):
     $$\mathbf{T}_{\text{anchor}} \leftarrow \mathbf{T}_{\text{px4}}(t_{\text{recover}}) \cdot \mathbf{T}_{\text{cuvslam}}(t_{\text{recover}})^{-1}$$
     This completely eliminates the $2.2\text{ m}$ recovery jump measured in simulation.
3. **Dynamic Covariance Injection ($\mathbf{R}_k$):**
   - Overwrite `cuVSLAM`'s uncalibrated covariance matrix with honest diagonal variances scaled by visual quality:
     - `HEALTHY` (texture $> 5\%$, $\|\Omega\| < 1.0\text{ rad/s}$): $\sigma_{\text{pos}}^2 = (0.02\text{ m})^2$, $\sigma_{\text{yaw}}^2 = (0.02\text{ rad})^2$.
     - `DEGRADED` (texture $1\%\text{--}5\%$ or high yaw rate): Inflate variance proportionally by $\alpha \in [2.0, 10.0]$ based on inverse texture ratio (inspired by MonoRace's $\sigma^2 \propto 1/N_c^2$) so EKF2 leans more heavily on the IMU during fast corner rotations.

---

### 3.4 Node 2: `course_planner` (Map-Driven A* & Topological Shortcutting)
**Purpose:** Parse the course geometry file (`v3` CAD mesh / JSON layout) to generate optimal waypoints without hard-coding coordinates.

**Algorithm Steps:**
1. **2.5D Horizontal Slice & Obstacle Inflation:**
   - Slice the 3D course model at flight altitude $z = 0.80\text{ m}$ to extract all wall segments and outer net boundaries ($6.096 \times 6.096\text{ m}$).
   - Project the 5 southern boulders ($r = 0.2285\text{ m}$) onto the 2D grid even though they are $0.533\text{ m}$ tall (below flight height). Expand boulder radii by the drone's lateral tolerance + VL53L1X downward cone footprint ($r_{\text{total}} = 0.2285 + 0.19 + 0.10\text{ m}$) so the rangefinder never pings a boulder top.
2. **Nav2 Costmap Discretization ($5\text{ cm}$ Grid):**
   - Build a $122 \times 122$ occupancy grid ($0.05\text{ m}$ resolution).
   - Compute Euclidean distance $d(c)$ from every cell $c$ to the nearest wall/net/boulder segment:
     - **Lethal / Inscribed Zone ($d \le 0.32\text{ m}$):** Impassable (`cost = INF`). Accounts for $0.28\text{ m}$ prop reach + $0.04\text{ m}$ safety margin. *(Note: For the $0.914\text{ m}$ Window passage, temporarily relax inscribed radius to $0.29\text{ m}$ along the window axis).*
     - **Exponential Inflation Decay ($0.32\text{ m} < d \le 0.80\text{ m}$):**
       $$\text{Cost}(d) = C_{\text{max}} \cdot \exp\left(-\lambda \cdot (d - r_{\text{inscribed}})\right)$$
       Forces the path directly down the centerlines of $0.93\text{ m}\text{--}1.07\text{ m}$ corridors.
3. **Sequential Checkpoint Goal Extraction (Programmatic from Map):**
   - Parse semantic regions directly from map geometry:
     1. `Start`: Launch mark $(-2.515, +0.914)$.
     2. `Goal 1`: First corridor north exit.
     3. `Goal 2`: Maze exit (Gate A).
     4. `Goal 3`: Dark Tunnel entry $\rightarrow$ centerline axis $\rightarrow$ South exit (strictly colinear along tunnel axis so no yaw turn occurs inside).
     5. `Goal 4`: Window center in wall C2a (`--profile=attack`) OR Gate B center (`--profile=bank`).
     6. `Goal 5`: Landing Pad center $(-2.042, -0.792)$.
4. **A* Search + Line-of-Sight Shortcutting:**
   - Run 8-connected (or Theta*) A* across sequential goals.
   - Prune redundant collinear/staircase grid nodes: replace subpath $(p_i \dots p_j)$ with segment $\overline{p_i p_j}$ iff every cell intersected by $\overline{p_i p_j}$ has a cost no greater than the maximum cost along the original grid subpath. Produces $\sim 19$ clean legs ($\sim 17\text{ m}$ total distance).
   - Support dynamic re-planning during flight if `map_corrector` updates wall segment offsets.

---

### 3.5 Node 3: `trajectory_follower` (50 Hz Led-Pose & Active Perception Gaze)
**Purpose:** Convert the piecewise-linear waypoint route into a smooth, zero-lag $50\text{ Hz}$ setpoint stream that prevents PID corner overshoot and keeps the OAK-D camera pointed at textured corridors.

**Core Logic:**
1. **Corner Arc Blending (Filleting):**
   - At each interior waypoint $\mathbf{W}_i$ where segment direction changes by angle $\theta_i$, insert a circular/clothoid fillet of radius $R_{\text{blend}} \in [0.20, 0.35]\text{ m}$ (clamped so the arc never violates the $0.32\text{ m}$ wall clearance).
2. **Jerk-Limited Velocity Profile:**
   - Compute 1D arc-length parameterization $s(t)$ along the blended path using PX4's configured kinematic limits:
     - $v_{\text{max}} = 1.0\text{ m/s}$ ($0.75\text{ m/s}$ at the first maze corner in `bank` profile to prevent PX4 v1.16.1 attitude overshoot).
     - Centripetal cornering limit: $v_{\text{corner}} = \sqrt{a_{\text{lat,max}} \cdot R_{\text{blend}}}$.
3. **Active Perception Look-Ahead Yaw (Anti-Starvation Gaze):**
   - **Never** point yaw tangent to current velocity when approaching a $90^\circ$ wall corner (facing a wall at $0.3\text{ m}$ starves `cuVSLAM`, dropping 390 of 543 frames).
   - Compute desired heading $\psi_{\text{ref}}(t)$ by pointing at a look-ahead point $s(t) + L_{\text{lookahead}}$ where **$L_{\text{lookahead}} = 0.80\text{ m}$**:
     $$\psi_{\text{ref}}(t) = \text{atan2}\left(y(s + 0.80) - y(s), \, x(s + 0.80) - x(s)\right)$$
   - **Tunnel Exception:** Inside the Dark Tunnel (`Goal 3`), lock $\psi_{\text{ref}}$ strictly along the tunnel centerline axis toward the south exit until the drone completely clears the tube.
4. **50 Hz Feed-Forward "Led Pose" Compensation (Bypassing PID Lag):**
   - Because `skybeta_flight_interface` only accepts position setpoints (and plain pose tracking lags by $\sim 17\text{ s}$ over the run), pre-lead the commanded position $\mathbf{p}_{\text{cmd}}(t)$ using the trajectory's instantaneous velocity $\mathbf{v}_{\text{ref}}(t)$ and acceleration $\mathbf{a}_{\text{ref}}(t)$:
     $$\mathbf{p}_{\text{cmd}}(t) = \mathbf{p}_{\text{ref}}(t) + \frac{\mathbf{v}_{\text{ref}}(t)}{k_p} + \frac{\mathbf{a}_{\text{ref}}(t)}{k_p \cdot k_v}$$
     where **$k_p = 0.95$** and **$k_v = 1.80$** match PX4's internal position and velocity P-gains.
   - *(Note: Do NOT add a cross-track error push in the follower; simulation A/B testing proved cross-track pushing increased wall touches from 1/20 to 4/20 under $50\text{ ms}$ latency).*
5. **Return Trip State Machine (`--enable-return`):**
   - Upon reaching Landing Pad (`Goal 5`):
     1. Descend, touch down inside the $1.524\text{ m}$ pad, and **disarm** (locks in Checkpoint 5 and full forward-run points).
     2. Wait $1.5\text{ s}$, **re-arm**, and take off vertically to **$z = 2.20\text{ m}$** (clears the $1.727\text{ m}$ center barrier wall; legal on return flight, stay below $3.048\text{ m}$ top net).
     3. Fly direct straight-line leg over the barrier to Launch Mark $(-2.515, +0.914)$.
     4. Descend, land within $0.61\text{ m}$ ($2\text{ ft}$) of Launch Mark, and disarm before the 3:00 clock expires for the **$1.25\times$ total score multiplier**.

---

### 3.6 Node 4: `map_corrector` (2D Depth Slice ICP & Layout Filter)
**Purpose:** Eliminate slow VIO drift ($\sim 8\text{ cm} \rightarrow \sim 2\text{ cm}$) and estimate if physical wall units were placed $5\text{--}10\text{ cm}$ off the published CAD file.

**Algorithm Steps:**
1. **Horizontal Depth Band Slicing & Net Frustum Culling:**
   - Take stereo depth from OAK-D S2, transform points inside the altitude band $z \in [0.65, 0.95]\text{ m}$ into the 2D horizontal plane using current roll/pitch from EKF2 (compensated for static mount tilt $\theta_{\text{cam}}$).
   - **Spatial Trust Gating:** Discard depth points beyond $3.5\text{ m}$, points with sparse neighbor density, and any point whose world projection falls within $0.15\text{ m}$ of or outside the $6.096\text{ m}$ mesh net boundary (prevents matching background room furniture seen through the net).
2. **2D Point-to-Line ICP / Iterated Kalman Filter:**
   - Match filtered 2D scan points against the v3 CAD wall line segments.
   - Apply a low-gain correction to $\mathbf{T}_{\text{anchor}}$ (bounded to max $2\text{ cm/s}$ drift correction so PX4 EKF2 never sees a velocity spike).
   - Maintain a per-wall-unit offset state $\Delta \mathbf{w}_k$ (like AlphaPilot gate offset estimation). If a wall unit is consistently observed shifted by $> 4\text{ cm}$, trigger `course_planner` to shift that obstacle segment and re-shortcut the remaining route. Reset all offsets on node restart (Rule 6.4).

---

## 4. Repository Structure

```text
dronehacks_2026/
├── PRD.md                          # This specification document
├── README.md                       # Setup, run instructions, and open-source credits (Rule 9.1/9.4)
├── contracts/
│   ├── course_v3_layout.json       # Published v3 CAD wall segments, boulders, window, pad, launch
│   └── system_constants.yaml       # Single source of truth for dimensions, gains, thresholds
├── launch/
│   ├── autonomy_run.launch.py      # Master launch: guard + planner + follower + map_corrector
│   └── bench_check.launch.py       # Props-off verification launch
├── src/
│   ├── skybeta_interface_patch/    # Forked flight interface (50Hz + Offboard-active gating)
│   ├── odom_guard/                 # Node 1: Texture/gyro/Mahalanobis gating & re-anchoring
│   ├── course_planner/             # Node 2: 2.5D slice, Nav2 costmap, A*, line-of-sight shortcut
│   ├── trajectory_follower/        # Node 3: Arc blending, 0.8m gaze, 50Hz Led-Pose, Return FSM
│   └── map_corrector/              # Node 4: Depth band slicer, net culling, ICP wall matcher
└── scripts/
    ├── day1_kit_checks.sh          # Automated CLI check for versions, DDS, QoS, clock, biases
    ├── survey_adjust_map.py        # CLI tool to input tape-measured wall offsets into layout JSON
    └── offline_iou_calibrator.py   # Post-flight log optimizer for camera pitch/roll extrinsics
```

---

## 5. Execution Timeline & Verification Checklist

### Phase 1: Bench & Kit Unblocking (Friday 7:00 PM – Midnight)
- [ ] **Save Parameters:** Backup factory PX4 params via QGC (`SN0001_factory.params`).
- [ ] **Set Estimator Keys:** Set `EKF2_EVA_NOISE = 0.01`, `EKF2_EV_QMIN = 0`, `EKF2_EV_DELAY = 0`.
- [ ] **Fast DDS Shared Memory Fix:** Configure XML profile with $> 4\text{ MB}$ shared memory segment so $1\text{ MB}$ stereo frames never drop under Best-Effort QoS.
- [ ] **Props-Off Carry Test (Frame Sign Check):**
  - Carry drone $+1.0\text{ m}$ North (up first corridor): verify `/fmu/out/vehicle_local_position.x` increases by $+1.0\text{ m}$.
  - Carry drone $+1.0\text{ m}$ East: verify `/fmu/out/vehicle_local_position.y` increases by $+1.0\text{ m}$.
  - Move drone $+0.5\text{ m}$ Up: verify `/fmu/out/vehicle_local_position.z` decreases by $-0.5\text{ m}$ (NED).
- [ ] **Lens Cover Bench Test (`odom_guard`):** Cover OAK-D lens for 3 seconds while walking the drone 1 meter, then uncover. Verify `odom_guard` transitions `HEALTHY -> LOST -> HEALTHY` and `/fmu/out/vehicle_local_position` exhibits **zero discontinuity jump**.
- [ ] **Physical Course Survey:** Tape-measure the Window frame (C2a), first maze corner, and narrowest bottom passage ($0.93\text{ m}$) against `course_v3_layout.json`. Update map via `survey_adjust_map.py`.

### Phase 2: Incremental Flight Progression (Saturday 9:00 AM – 7:00 PM)
- [ ] **Step 1:** $30\text{ s}$ settle $\rightarrow$ Position-mode manual hover $\rightarrow$ verify `EKF2_EV_DELAY = 0` gives minimum vision innovations in `.ulg` log.
- [ ] **Step 2:** Autonomous takeoff to $0.80\text{ m}$ (`HEIGHT_M = 0.73`) $\rightarrow 10\text{ s}$ Offboard hover $\rightarrow$ land. Verify zero takeoff yaw kick from patched interface.
- [ ] **Step 3:** Straight $2.0\text{ m}$ leg up First Corridor (Checkpoint 1).
- [ ] **Step 4:** Full Maze + Dark Tunnel (Checkpoints 1–3) with $8\text{ ms}$ exposure cap and `odom_guard` active.
- [ ] **Step 5:** Full forward course through Window + Landing Pad (`--profile=bank`). Target: **3 consecutive zero-touch finishes**.
- [ ] **Step 6:** Enable `--profile=attack` + `--enable-return` ($2.2\text{ m}$ barrier hop back to launch mark).

### Phase 3: Scored Blocks & Pitch (Saturday 10:00 PM – Sunday 4:00 PM)
- [ ] **Scored Block 1 (Sat Night):** Fly `--profile=bank` on Attempt 1 to lock a clean finished score on the leaderboard immediately. Switch to `--profile=attack --enable-return` once a finish is banked.
- [ ] **Sunday 12:30 PM Deadline:** Submit GitHub repo link, verify README credits (`skybeta` starter, NVIDIA Isaac ROS / `cuVSLAM`, Luxonis `depthai-ros`, PX4, ROS 2, Nav2 cost formulas, SciPy), and confirm zero hard-coded waypoints for Rule 9.5 check.
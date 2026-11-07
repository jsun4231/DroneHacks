# Deploying `dronehacks_lidar` on drone 1

These steps take the package from this repo to drone 1's team workspace (hostname `skybeta-01`),
following the kit's procedure (`drone-snapshot/home/dronehacks/skybeta/docs/competitor_guide.md`
§5). Never log into or deploy to another drone: that gets a team disqualified (AGENTS.md §1).

## Once: install

On the drone, in the host shell, from a checkout of this repo (a Linux checkout keeps the
scripts' LF line endings):

```bash
~/skybeta/host/run.sh build --init          # first time only: writes ~/team_ws/Dockerfile
bash depthcam/deploy/install_on_drone.sh    # copies the package to ~/team_ws/src/dronehacks_lidar, adds pyserial
~/skybeta/host/run.sh build                 # the team image, with pyserial
```

The script refuses to run on any host but `skybeta-01`. `depthcam/` is the package root:
`package.xml`, `setup.py`, `launch/`, the Python packages, the map, the routes and the config.
The stack's `competitor` window builds the workspace with `team_build.sh` when the stack starts.
Don't put `px4_msgs` in the workspace, because the image carries the matching one.

## Once: the lidar

- Give the MS200K's USB adapter a fixed name, `/dev/ms200`, with a udev rule on the host
  (course/README.md, *Wiring*).
- The stack container runs `--privileged` with `/dev/bus/usb` mounted. Check the name inside
  it: `~/skybeta/host/run.sh shell` then `ls -l /dev/ms200`.
- The mount values in `config/pose_fusion.example.yaml` are the ones from past tests: yaw offset
  87.4°, clockwise, scan plane 0.035 m above the body centre. If the lidar is ever moved,
  re-measure them, because every degree is about 1.7 cm of error per metre.

## The route

Two routes ship with the package, both planned from the STEP-model map
(`course/maps/course_v3_map.json`, extracted from `dronehacks-course-v3.step`) through the
ground-truth checkpoints (`DRONEHACKS.md` §4, `course/maps/course_v3_rules.json`):

| Route | CP4 | Length | Tightest room beyond the 5 cm margin |
|---|---|---|---|
| `window` (default) | through C2a's window (+5 bonus) | 15.5 m | +3 mm |
| `gate_b` | through Gate B | 16.2 m | +3 mm |

To replan after a change to the map or the rules, run `python -m follower.make_route` from
`depthcam/` and redeploy.

## Every run

1. **Choose the run's lidar policy and route.** In `/etc/skybeta/drone.env` (`sudoedit`), set
   `COMPETITOR_PKG=dronehacks_lidar` and `COMPETITOR_LAUNCH` to one of these:

   | `COMPETITOR_LAUNCH` | Lidar | Route |
   |---|---|---|
   | `lidar_nav_allowed.launch.py` | allowed | window |
   | `lidar_nav_allowed_gate_b.launch.py` | allowed | Gate B |
   | `lidar_nav_forbidden.launch.py` | forbidden | window |
   | `lidar_nav_forbidden_gate_b.launch.py` | forbidden | Gate B |

   The kit passes only these two keys into the container, so the choice lives in the file name.
2. **Put the drone on the launch mark, facing north, and restart the stack:**
   `docker rm -f skybeta-stack` (it is back in about 10 s). The restart re-zeroes cuVSLAM's
   origin there, which is the frame the route is flown in. If you skip it, the origin is
   wherever the stack started (at power-on). The follower's pre-takeoff check then blocks the
   flight, unless the lidar is allowed and corrects the offset.
3. **Wait for ready.** `/pose_fusion/ready` must be `true`; it always is when the lidar is
   forbidden. `/follower/status` must say `WAIT_FOR_OFFBOARD`. `/pose_fusion/lidar_policy`
   shows the policy (latched).
4. **The pilot arms, then switches to Position.** `offboard_active` goes true, and the follower
   checks the start. If the drone is on the route start at the planned heading, and pose_fusion
   is ready, it takes off and flies. Otherwise it waits up to 20 s, then blocks: it sends nothing,
   the vehicle holds at the interface's 0.3 m floor, and the pilot takes over.
5. **The run:** CP1 → CP4, hold over the pad, land by velocity setpoint. PX4 disarms 2 s after
   touchdown (`COM_DISARM_LAND`). `/follower/status` lists the checkpoints crossed.

By hand instead of the `competitor` window: stop what the window runs (attach, select it,
Ctrl-C; two flight interfaces fight), then
`ros2 launch dronehacks_lidar lidar_nav.launch.py lidar_policy:=allowed route:=window`.

## Before the first flight with the props on

- Carry test: carry the drone by hand from the launch mark. `/pose_fusion/odometry` must follow
  it in the tracker frame (x = north, y = west).
- Props off: arm and switch to Position. The follower must leave `WAIT_FOR_OFFBOARD`, and the
  setpoints on `/skybeta/setpoint/pose` must climb to 0.73 m.
- First hover, then the first leg, with the pilot ready on the mode switch.

# DAC-SFC deployment validation

This branch adds a deployable DAC-SFC path to the existing EGO-Planner-v2
system. The real-vehicle study is intentionally a feasibility study, not a new
controlled comparison.

## Runtime pipeline

The planner executes:

1. direct visibility check or the existing 26-connected A* search;
2. farthest-visible route simplification with a maximum segment length;
3. an unoptimized quintic MINCO deformation probe using the current P/V/A;
4. proximity-weighted CSGN and spectrum compression;
5. one Active-Witness corridor per sparse route segment;
6. the paper-production GCOPTER backend;
7. an independent sampled collision check against the inflated `GridMap`;
8. conversion to the existing `poly_traj::Trajectory`/trajectory-server format.

The DAC-SFC algorithm headers come from production commit
`ddfe6c710cc80f6686da2906e427acfe52b4a77c` of the companion GCOPTER
repository. The dormant legacy `traj_opt/include/optimizer/gcopter.hpp` is not
used.

## Safe validation sequence

Both launch configurations enable DAC-SFC in shadow mode. In this mode the
complete pipeline, including GCOPTER, runs and is logged, but the original EGO
trajectory remains in control.

Build in the ROS workspace as usual:

```bash
cd src/main_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

After sourcing the PX4 Gazebo environment and adding PX4_Firmware to
`ROS_PACKAGE_PATH`, start the complete single-vehicle simulation stack:

```bash
roslaunch ego_planner dac_sfc_single_sim.launch
```

This entry point launches Gazebo, PX4 SITL, the vehicle model, MAVROS,
`px4ctrl`, the planner, trajectory server and RViz. It intentionally does not
use the external `px4/single_vehicle.launch`, because that file is not part of
this repository and may be generated or locally modified.

The launcher follows the repository's XTDrone configuration and uses
`single_vehicle_spawn_xtd.launch`, the `iris_realsense_camera` model and the
same vehicle-0 MAVLink ports as `px4/multi_vehicle.launch`.

During a run, check:

```bash
rostopic echo /drone_0_ego_planner_node/a_star_list
tail -f /tmp/dac_sfc_deployment/dac_sfc_deployment.csv
```

The ROS console emits one compact line per attempt:

```text
[DAC-SFC] mode=shadow success=1 route=42->7 corridors=6 faces=61 geo/call=1037 total_ms=...
```

After repeated shadow runs are successful and the sparse route is correct in
RViz, opt into DAC-SFC output from the launch command. For the first active
simulation, use conservative dynamic limits:

```bash
roslaunch ego_planner single_run_in_gazebo.launch \
  dac_sfc_shadow_only:=false max_vel:=0.5 max_acc:=1.0
```

The complete-stack launcher forwards the same arguments:

```bash
roslaunch ego_planner dac_sfc_single_sim.launch \
  dac_sfc_shadow_only:=false max_vel:=0.5 max_acc:=1.0
```

Omitting `dac_sfc_shadow_only:=false` keeps the safe shadow default. Keep
`fallback_to_ego=true` during initial simulation and onboard tests.

## Simulation in place of unavailable onboard topics

`advanced_param_gazebo.xml` labels all records as `simulation` and continues to
use the current simulated depth/pose/odometry remaps. No onboard-only topic is
required by DAC-SFC itself: it consumes the same `GridMap`, current state and
local target already available to EGO-Planner.

`advanced_param_exp.xml` labels records as `onboard`. It is also kept in shadow
mode for the first dry runs. Before active flight, confirm the real topic remaps
and replace the placeholder airframe mass, thrust, drag, body-rate and tilt
limits.

## Recorded evidence

The logger writes `/tmp/dac_sfc_deployment/dac_sfc_deployment.csv`. Each row
contains:

- source and mode: simulation/onboard, shadow/active, fallback use;
- feasibility: pipeline success, trajectory activation and sampled collision
  status;
- A* and route: raw/sparse point counts, lengths, search and shortcut time;
- algorithm latency: guide, CSGN, corridor, GCOPTER setup/optimization and total
  time;
- workload: obstacle points, corridor count, total faces and exact geometric
  face evaluations per objective/gradient call;
- trajectory output: duration, maximum velocity and acceleration;
- diagnostics: Active-Witness rounds, removed redundant faces, valid CSGN
  pieces, anisotropy, final cost and corridor violation.

Summarize one or more logs with:

```bash
python3 src/main_ws/src/planner/dac_sfc/tools/summarize_deployment.py \
  /tmp/dac_sfc_deployment/dac_sfc_deployment.csv
```

For the paper's short deployment subsection, report the number of runs,
pipeline success rate, fallback count, median/95th-percentile total latency,
median A*/CSGN/corridor/optimization latency, corridor/face workload, and the
observed trajectory velocity/acceleration envelopes. Do not present these rows
as a controlled algorithm comparison.

## Activation gates

Do not disable shadow mode until all of the following hold in simulation:

- no `route_*`, `csgn_*`, `active_witness_corridor`, `corridor_overlap`,
  `gcopter_*` or `corridor_violation` failures;
- `sampled_collision_free=1` for every trajectory intended for execution;
- total planning time remains below the available replanning horizon;
- vehicle mass, thrust, drag, body-rate and tilt parameters match the airframe;
- EGO fallback and emergency stop are still enabled and tested.

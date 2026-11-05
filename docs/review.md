# Code review and revision

## Original implementation

The original `SimNode.cpp` is preserved in the repository's Git history. Work for this revision was performed in a separate local clone.

The original intention was to alter the trajectory around an obstacle, rather than merely stop. Its point-wise repulsion had several problems:

- The node subscribed and published to the same `/cmd_vel` topic, so output could be consumed as a new input command.
- Its nominal loop frequency was 10,000,000,000 Hz, which effectively removed a useful control-period bound.
- Distance was squared, while the threshold's meaning was implicit. The inverse squared-distance term was unbounded near zero, and corrections grew with point density.
- No robot footprint, predicted path, braking allowance, or output limits were used.
- Laser data and commands could become stale. Failed transforms did not invalidate old geometry.
- Commands were manually cleared after one loop, rather than retained for a defined validity interval.
- The package declaration omitted dependencies used by the source and the CMake file contained unnecessary message generation and Eigen requirements.

## Revised design

The new core samples constant-curvature trajectories for a circular differential-drive robot. It is intentionally smaller than a navigation stack and has no map or goal input. It filters a desired forward velocity and yaw rate using current obstacle points in the base frame.

The collision radius is `robot_radius + safety_margin`, with another half-step translation allowance to cover distances between rollout samples. Candidates additionally require a straight-ahead initial braking allowance `v * reaction_time + v² / (2 * braking_deceleration)` for returns in the forward footprint strip. The ROS adapter also supplies scan acquisition age; obstacle clearance is reduced by a bound on base translation and rotation since acquisition, using the configured speed limits. This is a conservative screening rule under the static-obstacle model, not a proof of dynamic safety.

The candidate score combines endpoint agreement with the desired arc, turning deviation, change from the previous turn command, clearance, and forward progress. A stopped candidate is penalized when the user requests forward motion, so a clear bypass is preferred over simply waiting. Speed never exceeds the positive requested speed or the configured limit. A bypass-side latch avoids repeatedly reversing steering at a centered obstacle; it is released when the desired arc has ample clearance or the filter is reset. This can be suboptimal in tight/concave environments.

ROS-specific concerns are isolated in the adapter: distinct remappable topics, queue size 1, a 20 Hz wall-clock timer, scan and command freshness, laser projection into the base frame, range/coverage checks, malformed-input handling, and bounded parameters. Zero output is published periodically when input is unavailable. The controller does not impose a velocity acceleration profile; physical tracking and emergency stopping depend on the base.

## Demo provenance

`tools/simulate.cpp` calls the same `Controller::filter` function as the ROS adapter. It raycasts a 360-beam, 8 m scanner against known circular obstacles and integrates the selected differential-drive command every 0.10 s. The controller receives the scan points, not obstacle centers, radii, world coordinates, or the goal. Full geometry is used separately to check actual clearance.

The external intent policy turns toward `(6.5, 0)` from `(0, 0)` at up to 0.55 m/s. A circle centered at `(2.5, 0)` with radius 0.45 m obstructs the direct line. The robot radius is 0.20 m with a 0.10 m margin. The goal is reached within a 0.15 m tolerance. No noise, wheel dynamics or moving obstacles are included.

The GIF is rendered exclusively from the resulting CSV; it is not a video recorded in ROS, Stage, or a hardware environment. The highlighted outer ring is the obstacle radius plus robot radius plus margin, not a separate measured obstacle.

## Checks

- Standalone release build with compiler warnings enabled.
- Unit checks for clear-path preservation, moving avoidance, a completely blocked surround, zero/reverse/nonfinite commands, footprint intrusion, bounds, invalid configuration, and stale/invalid input guards.
- Four closed-loop scenarios: clear space, centered obstacle, offset obstacle, and two obstacles. Each requires goal arrival and at least 0.10 m actual surface clearance.
- ROS CI build in `ros:noetic-ros-base-focal` plus synthetic-scan integration checks for pass-through, moving avoidance, missing/invalid scans, both watchdog timeouts and recovery.

The local development machine has no ROS installation. ROS compile and topic integration checks therefore run in GitHub Actions; the standalone core and visualization are executed locally. Hardware testing is not claimed. The subsequent Stage integration is described below.

## Original assignment and Stage integration

The [original course assignment](https://gitlab.com/grisetti/labiagi_2020_21/-/blob/master/projects_preliminary/project_collision_avoidance.txt) specifies a laser scan and desired velocity as inputs, with a collision-avoiding velocity as output, preferably deflecting toward free space. It sketches a sum of repulsive obstacle forces and the desired command. This revision preserves the input/output contract and deflection objective, but uses sampled trajectories instead of that force-summing sketch. The repository should not be read as an unchanged implementation of the suggested force law.

`launch/stage_demo.launch` and `worlds/avoidance.world` add the complete simulator path: Stage laser/TF -> ROS adapter -> shared controller -> Stage motion. `tools/stage_trial.py` supplies upstream goal-following intent, records actual Stage ground truth and node outputs, and independently checks arrival, moving avoidance, output limits and clearance against the known box geometry. The Stage robot is a 0.28 m square, contained by the configured 0.20 m radius. The obstacle is a 0.90 m square at `(2.5, 0)`; the goal is `(6.5, 0)`.

The automated Stage check uses a 0.095 m clearance threshold to allow 5 mm of verification tolerance around the nominal 0.10 m margin. It is an integration test in a simulator, not a hardware safety certification. It does not reuse the idealized simulator's circle geometry or its dynamics. It runs under ROS Noetic in Linux CI because the development Mac has no ROS runtime. The original GIF remains labeled as the standalone 2D demo.

# Pending Tasks - Future Development

> Tasks planned for immediate and long-term development

---

## Immediate Priority (Before Pool Testing)

### SITL Validation 🖥️
**Status:** NEXT UP

- [ ] Connect Gazebo ArduSub simulator to ROS 2 pipeline
- [ ] Test V2 features in simulation:
  - [ ] Velocity estimator with gravity compensation
  - [ ] Convergence gates for waypoint detection
  - [ ] Active braking near waypoints
  - [ ] Cascade position/velocity control
  - [ ] Gain scheduling at different speeds
- [ ] Run test missions:
  - [ ] Square pattern with 90° turns
  - [ ] Circle pattern with continuous yaw
  - [ ] Complex multi-waypoint maneuvers
- [ ] Parameter tuning:
  - [ ] Velocity estimator alpha filter
  - [ ] Convergence gate thresholds
  - [ ] Active braking gains
  - [ ] Cascade controller PIDs
  - [ ] Gain scheduling curves
- [ ] Record performance metrics (overshoot, settling time, accuracy)
- [ ] Document tuned parameters for pool testing

**Timeline:** This week (April 4-7, 2026)

---

## Short Term (Pool Testing Phase)

### Pool Testing Preparation 🏊
**Status:** Waiting for SITL completion

**Pre-Pool Checklist:**
- [ ] Verify Pixhawk connection and telemetry
- [ ] Check thruster response to all DOF commands
- [ ] Validate camera feed quality (forward, downward)
- [ ] Prepare logging and data collection
- [ ] Load SITL-tuned parameters
- [ ] Create pool test procedure checklist
- [ ] Prepare emergency procedures

**Pool Test Sequence:**

#### Session 1: Basic Validation (1-2 hours)
- [ ] Shallow water tests (0.5m depth)
- [ ] Thruster calibration verification
- [ ] Sensor health checks (IMU, pressure, compass)
- [ ] RC override emergency testing

#### Session 2: Control Validation (2-3 hours)
- [ ] Depth holding tests:
  - [ ] 0.3m depth (5 minute hold)
  - [ ] 0.5m depth (5 minute hold)
  - [ ] 1.0m depth (5 minute hold)
  - [ ] 2.0m depth (5 minute hold)
- [ ] Yaw control tests:
  - [ ] 90° turn precision
  - [ ] 180° turn precision
  - [ ] 270° turn precision
  - [ ] Continuous slow rotation
- [ ] Stability tests:
  - [ ] Hold position in current/disturbance
  - [ ] Recovery from manual displacement

#### Session 3: Mission Validation (2-3 hours)
- [ ] Square mission (1m × 1m pattern)
- [ ] Circle mission (0.5m radius)
- [ ] Complex waypoint sequence
- [ ] Vision-based gate alignment
- [ ] Emergency surface procedure
- [ ] Battery compensation validation (test at 16.8V, 14.4V, 12.6V)

**Data Collection:**
- [ ] Log all sensor data (IMU, pressure, compass, PWM outputs)
- [ ] Record video of all test runs
- [ ] Document parameter changes and results
- [ ] Measure performance metrics vs SITL

**Timeline:** April 8-15, 2026 (pending pool availability)

---

## Medium Term (Post-Pool Development)

### DVL Integration - Nortek Nucleus 1000 📡
**Status:** Hardware available, driver needed

**Phase 1: Driver Development**
- [ ] Research ROS 2 DVL driver options:
  - [ ] Check for existing Nortek ROS 2 drivers
  - [ ] Evaluate community drivers
  - [ ] Consider writing custom driver
- [ ] Implement MAVLink DVL driver:
  - [ ] Parse MAVLink `VISION_POSITION_DELTA` messages
  - [ ] Extract velocity (vx, vy, vz) from DVL
  - [ ] Publish to `/dvl/velocity` topic
  - [ ] Add bottom lock detection
- [ ] Integrate with velocity estimator:
  - [ ] Fuse DVL velocity with IMU acceleration
  - [ ] Implement fallback to IMU-only mode
  - [ ] Handle bottom lock loss gracefully

**Phase 2: Position Estimation**
- [ ] Dead reckoning from DVL velocity:
  - [ ] Integrate velocity to position
  - [ ] Reset on known waypoints
  - [ ] Publish `/dvl/position` estimate
- [ ] Feed into Pixhawk EKF (if possible):
  - [ ] MAVLink `VISION_POSITION_ESTIMATE` messages
  - [ ] Or use software UKF in ROS 2

**Phase 3: Navigation**
- [ ] Implement `NavigateToWaypoint` YASMIN state:
  - [ ] Closed-loop position control
  - [ ] Target waypoint (x, y, z)
  - [ ] Convergence detection
- [ ] Test waypoint navigation:
  - [ ] Single waypoint accuracy (±20cm target)
  - [ ] Multi-waypoint sequence
  - [ ] Return-home capability

**Phase 4: Mission Integration**
- [ ] Update missions to use waypoint navigation
- [ ] Test in pool with DVL
- [ ] Validate accuracy over 10m traverse

**Timeline:** April 15 - May 1, 2026

---

### Vision-Based Autonomy Enhancements 📷

**Custom YOLO Models:**
- [ ] Collect training data from pool sessions:
  - [ ] Gate poles and crossbar
  - [ ] Red and white slalom pipes
  - [ ] Bin symbols (downward camera)
  - [ ] Path markers (orange, on floor)
  - [ ] Torpedo targets
- [ ] Augment with synthetic data (varied water conditions)
- [ ] Train YOLO11n with RoboSub prop classes:
  - [ ] `gate_pole`, `gate_crossbar`
  - [ ] `red_pipe`, `white_pipe`
  - [ ] `bin_symbol_*` (chevron, taurus, scorpio, etc.)
  - [ ] `torpedo_target`
  - [ ] `octagon_frame`
  - [ ] `path_marker`
- [ ] Export to TensorRT for Jetson Orin Nano
- [ ] Target mAP > 0.8 on validation set

**Multi-Object Tracking:**
- [ ] Extend `KalmanObjectTracker` for N simultaneous tracks
- [ ] Implement Hungarian algorithm for detection-track association
- [ ] Publish `TrackedObjectArray` with track IDs
- [ ] Test with slalom task (track 2+ pipes simultaneously)

**Multi-Camera Setup:**
- [ ] Mount downward-facing USB camera
- [ ] Configure `camera_manager` for dual operation
- [ ] Test USB bandwidth with dual 640×480@30fps
- [ ] Integrate downward camera into vision pipeline

**Timeline:** April 20 - May 15, 2026

---

### State Machine & Mission Development 🎯

**RoboSub 2026 Task State Machines:**
- [ ] Task 1: Begin Assessment (Gate) — ✅ Already exists
- [ ] Task 2: Avoid Debris (Slalom):
  - [ ] Multi-pipe detection and tracking
  - [ ] Slalom waypoint planning
  - [ ] Obstacle avoidance logic
- [ ] Task 3: Recon (Bins):
  - [ ] Downward camera symbol detection
  - [ ] Bin alignment and hover
  - [ ] Dropper integration
- [ ] Task 4: Deploy (Torpedoes):
  - [ ] Torpedo target detection
  - [ ] Alignment and range estimation
  - [ ] Launcher integration
  - [ ] Acoustic pinger integration (if available)
- [ ] Task 5: Resupply (Octagon):
  - [ ] Octagon frame detection
  - [ ] Surface maneuver
  - [ ] Grabber pickup sequence
  - [ ] Acoustic pinger integration (if available)
- [ ] Task 6: Return Home (Gate) — ✅ Reuse Task 1 with DVL

**Full Competition Mission:**
- [ ] Chain all task state machines
- [ ] Implement task timeout and skip logic
- [ ] Add mission abort/recovery procedures
- [ ] Test complete mission end-to-end

**Timeline:** May 1 - June 1, 2026

---

## Long Term (Future Enhancements)

### Simulation & Testing Infrastructure 🧪

**Gazebo Integration:**
- [ ] Connect Gazebo SITL to ROS 2 pipeline
- [ ] Create test world with RoboSub competition props
- [ ] Implement rosbag2 recording in launch files
- [ ] Run YASMIN missions in simulation before pool

**Testing Infrastructure:**
- [ ] Unit tests for state machine transitions
- [ ] Integration tests with simulation
- [ ] Replay-and-evaluate workflow for rosbags
- [ ] Continuous integration (CI) for build validation

**Timeline:** May - June 2026

---

### Acoustic Pinger Integration 🔊

**Research Phase:**
- [ ] Research hydrophone hardware options
- [ ] Evaluate MUSIC algorithm on Jetson Orin Nano
- [ ] Assess computational requirements

**Implementation (if feasible):**
- [ ] Acquire hydrophone array
- [ ] Implement DOA (Direction of Arrival) algorithm
- [ ] Publish bearing to pinger on ROS 2 topic
- [ ] Integrate with torpedo and octagon state machines

**Timeline:** June 2026 (stretch goal)

---

### Dashboard & Monitoring 📊

**Web Dashboard:**
- [ ] WebSocket bridge for ROS 2 topics
- [ ] React dashboard with:
  - [ ] Real-time vehicle state (depth, heading, battery, mode)
  - [ ] Camera feeds with detection overlays
  - [ ] Mission progress indicator
  - [ ] YASMIN state visualization
  - [ ] Diagnostics and error messages

**Timeline:** May - June 2026

---

### Actuator Integration 🤖

**Torpedo Launcher:**
- [ ] Test launcher mechanism
- [ ] Integrate with servo control
- [ ] Add to `DuburiClient` API
- [ ] Create YASMIN state for torpedo firing

**Marker Dropper:**
- [ ] Test dropper mechanism
- [ ] Integrate with servo control
- [ ] Add to `DuburiClient` API
- [ ] Create YASMIN state for marker drop

**Grabber Enhancement:**
- [ ] Test pickup sequence in pool
- [ ] Tune servo timing and force
- [ ] Add object detection feedback

**Timeline:** May 2026

---

## Competition Preparation (June - July 2026)

### Pre-Competition Validation
- [ ] Full mission dry-run in pool
- [ ] Backup system testing (redundant components)
- [ ] Travel logistics and equipment checklist
- [ ] Team training on emergency procedures
- [ ] Documentation of all procedures

### Competition Day Strategy
**Priority order by points-per-time:**
1. ✅ Gate (easy, fast) — guaranteed points
2. Square/Circle validation — baseline autonomy
3. Return Home (reuses Gate) — bonus points
4. Slalom (if tuned) — medium difficulty
5. Bins (if downward cam ready) — skip if not
6. Torpedoes/Octagon (require pinger) — skip if hardware not ready

---

## Post-RoboSub 2026 Vision

- [ ] DVL-based SLAM (Simultaneous Localization and Mapping)
- [ ] Multi-vehicle coordination
- [ ] Machine learning for adaptive control
- [ ] Advanced mission planning with optimization
- [ ] Open-source release of full stack
- [ ] Deep dive capabilities (pressure hull validation)

---

## Risk Mitigation

| Risk | Mitigation Strategy |
|------|-------------------|
| SITL validation delays | Start pool testing with basic parameters |
| DVL integration difficulties | Maintain IMU-only fallback mode |
| Pool time insufficient | Prioritize SITL simulation for development |
| YOLO performance issues | Use heavy data augmentation, multiple models |
| Actuator failures | Implement graceful degradation, task skip logic |
| Competition day sensor failures | Extensive redundancy and fallback modes |

---

**Last Updated:** April 3, 2026

# PX4 Configuration for Home Test Drone

## Loading Parameters

### Via QGroundControl
1. Connect Pixhawk 6C to QGroundControl
2. Navigate to: **Vehicle Setup** → **Parameters**
3. Click **Tools** → **Load from file**
4. Select `params.params`
5. Click **OK** to apply all parameters
6. Reboot flight controller

### Via MAVSDK (Programmatic)
```cpp
// Load parameters during initialization
auto result = system.load_params("config/px4/params.params");
```

## Key Configuration Notes

### Vision-Based Position Estimation

This configuration uses the T265 tracking camera for position estimation in GPS-denied environments:

- **EKF2_AID_MASK = 24**: Enables vision position and yaw fusion
- **EKF2_HGT_MODE = 3**: Uses vision for altitude (instead of barometer)
- Vision data must be published to PX4 via MAVLink `VISION_POSITION_ESTIMATE` or `ODOMETRY` messages

### Indoor Flight Tuning

Parameters are conservatively tuned for safe indoor operation:

- **Reduced velocities**: Max 1.0 m/s horizontal, 0.5 m/s vertical
- **Smooth motion**: Lower cruise speed (0.5 m/s) for predictable behavior
- **Gentle takeoff/landing**: 0.5 m/s takeoff, 0.3 m/s landing

### Offboard Control Settings

Configured for autonomous offboard control via MAVSDK:

- **COM_OBL_ACT = 1**: Return-to-launch on offboard signal loss
- **COM_OBL_RC_ACT = 0**: Allows offboard without RC transmitter
- **COM_OF_LOSS_T = 1.0**: 1-second timeout before failsafe triggers

### Safety Features

- **Battery failsafes**: RTL at 25% battery, land at 15%
- **Geofence**: Configurable boundaries (10m horizontal, 5m vertical default)
- **Offboard timeout**: Returns to launch if control lost for >1 second

## Tuning for Your Drone

These parameters are starting points. You may need to adjust based on:

### Frame Configuration
- Different propeller sizes → adjust rate controller gains
- Different weight → adjust MPC velocity/acceleration limits
- Different motor KV → may need different rate gains

### Sensor Placement
- **EKF2_EV_POS_X/Y/Z**: Adjust if T265 is not at center of mass
- Measure offset from flight controller to T265 camera

### Flight Characteristics
If drone feels:
- **Sluggish**: Increase `MPC_XY_VEL_P_ACC`, `MPC_Z_VEL_P_ACC`
- **Oscillating**: Decrease P gains, increase D gains
- **Drifting**: Check vision quality, may need to tune EKF2 vision noise params

## Testing Procedure

1. **Tethered hover test**: Verify vision position hold
2. **Manual position mode**: Test response to stick inputs
3. **Small offboard moves**: Test MAVSDK control with 0.5m moves
4. **Gradual expansion**: Increase complexity as confidence builds

## Parameter Backup

Always backup working parameters:
```bash
# Export current params from QGroundControl
# Save to: config/px4/params_backup_YYYYMMDD.params
```

## References

- [PX4 Parameter Reference](https://docs.px4.io/main/en/advanced_config/parameter_reference.html)
- [Vision/MoCap Setup](https://docs.px4.io/main/en/ros/external_position_estimation.html)
- [Offboard Control](https://docs.px4.io/main/en/flight_modes/offboard.html)

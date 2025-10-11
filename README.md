# FTC Team 22689
## Connections
### Control Hub
- USB 3.0: Webcam 
- Motor Port 0: Front Left Motor
- Motor Port 1: Front Right Motor
- Motor Port 2: Back Left Motor
- Motor Port 3: Back Right Motor
- I2C0: (IMU)
- I2C1: Colour Sensor
- I2C2: Pinpoint Computer
- RS485: Expansion Hub
### Expansion Hub
- Motor Port 0: Outtake Motor
- Motor Port 1: Intake Motor
- RS485: Control Hub
## TODO
### General
- [X] Get motors working
- [ ] Get servos working
- [X] Get colour sensor working
- [X] Get webcam working
- [ ] Proper turning and movement
- [ ] Use `@Config` to allow for variables to be updated in FTC Dashboard
- [ ] Rumble (vibrate) controllers depending on how much time is left (eg. 30s left)
- [ ] Hard code start locations and team alliance during Auto `init_loop`
- [ ] Sorting?
- [ ] Algorithm to calculate motor power needed to launch balls (polynomial curve fitting?)
- [ ] Optimize control loop (read only necessary sensors, ...)
- [ ] Finish `MainTeleOp`
- [ ] Finish `MainAuto`
### Road Runner
- [ ] Configure Road Runner for pinpoint computer + imu
- [ ] Road Runner tuning
- [ ] Integrate road runner in TeleOp and Auto
- [ ] Sensor fusion with vision (AprilTag pose estimation)
### Vision/Webcam
- [X] AprilTag detection
- [ ] Read obelisk at beginning of Auto
- [ ] Configure AprilTag detection for a variety of conditions (dim, bright, ...)
- [ ] AprilTag pose estimation
- [ ] Detect balls from webcam and get their coordinates
- [ ] Detect balls in the rails
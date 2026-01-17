# MainTeleOp
This is the main teleoperated (TeleOp) program for our FTC robot.
# BlueBottomAuto
This is the autonomous program for the blue alliance starting at the bottom side.
# Lift
Manages the lift mechanism of the robot for use during parking.
## Methods
- `up()`: Moves the lift up.
- `down()`: Moves the lift down.
- `update()`: Update method to be called in the main loop.
- `getTargetTicks()`: Returns the target position of the lift in ticks.
- `setTargetTicks(int ticks)`: Sets the target position of the lift in ticks.
- `getLiftTicks()`: Returns the current position of the left lift motor in ticks.
- `getRightTicks()`: Returns the current position of the right lift motor in ticks.
## Constants
- `posP`, `posI`, `posD` - PID coefficients for lift position control.
- `syncP`, `syncI`, `syncD` - PID coefficients for synchronizing left and right lift motors.
- `kHold` - Feedforward constant to hold the lift in place.
- `upIncrement` - Number of ticks to move the lift up per command.
- `downIncrement` - Number of ticks to move the lift down per command.
- `maxTicks` - Maximum height of the lift in ticks.
- `minTicks` - Minimum height of the lift in ticks.
# BallLaunch
Manages the ball launching mechanism of the robot.
## Methods
- `update()`: Update method to be called in the main loop.
- `launch()`: Tries to launch a ball. Return true if successful.
- `getTargetVelocity()`: Returns the target velocity for the outtake motor.
- `setTargetVelocity(double velocity)`: Sets the target velocity for the outtake motor.
## Important Variables
- `launchCount` - Number of balls to be launched.
- `forceLaunch` - If true, outtake will begin spinning up and prepare for launching regardless of `launchCount`.
## Constants
- `reloadTime` - Time in milliseconds required between launches.
- `servoStartPosition` - Starting position of the launch servo.
- `servoLaunchPosition` - Ending position of the launch servo.
- `velocityTolerance` - Acceptable error margin for outtake motor velocity.
# Intake
Manages the intake mechanism of the robot.
## Methods
- `pullIn()`: Activates the intake to pull in Artifacts.
- `pushOut()`: Activates the intake to push out Artifacts.
- `stop()`: Stops the intake mechanism.
## Constants
- `power` - Motor power for the intake motor when activated.
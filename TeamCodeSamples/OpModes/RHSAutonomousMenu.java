/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 */

@Autonomous(name = "Auto Config Menu", group = "FreightBot")
//@Disabled
public class RHSAutonomousMenu extends OpMode {
    static final double WHEEL_DIAMETER = 4;
    static final double ROTOR_SPEED = 2400;
    static final double DRIVE_SPEED = .8;
    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    // Declare OpMode members.
    Servo handServo;
    BNO055IMU imu;
    DistanceSensor frontDistance, leftDistance, rightDistance, backDistance;
    ColorSensor colorSensor;
    DcMotorEx arm;
    DcMotorEx rotor;
    // Setup a variable for each drive wheel to save power level for telemetry
    private DcMotorEx leftRear = null;
    private DcMotorEx rightRear = null;
    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    // AutonomousConfiguration Menu options
    private AutonomousConfiguration autoConfig;
    private boolean optionsSelected = false;
    private AutonomousConfiguration.AllianceColor alliance;
    private AutonomousConfiguration.StartPosition startPosition;
    private AutonomousConfiguration.ParkLocation parkLocation;
    private AutonomousConfiguration.DeliverDuck deliverDuck;
    private AutonomousConfiguration.DeliverFreight deliverFreight;
    private int delayStartSeconds;
    // Loop cycle time stats variables
    private ElapsedTime mStateTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);  // Time into current state
    private State mCurrentState;    // Current State Machine State.

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftRear = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        rightRear = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        leftFront = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        rightFront = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        autoConfig = new AutonomousConfiguration(gamepad1, telemetry);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        handServo = hardwareMap.get(Servo.class, "hand_servo");
        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";

        imu.initialize(parameters);

        arm = hardwareMap.get(DcMotorEx.class, "arm_motor");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rotor = hardwareMap.get(DcMotorEx.class, "rotor_motor");
        rotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        mCurrentState = State.STATE_INITIAL;
        // Tell the driver that initialization is complete.
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("TPS", "(%3.2f)", getTicksPerSecond());
//        telemetry.addData("TicksPerInches", "(%d)", getInchesToTicks(23));
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     * Be aware that in an iterative opMode the AutonomousConfiguration code will not
     * wait for you to press the gamepad start button before you press the app Start.
     */
    @Override
    public void init_loop() {
        if (!optionsSelected) {
            optionsSelected = autoConfig.GetOptions();
        }

        if (optionsSelected) {
            saveSelectedOptions();
            ShowSelectedOptions();
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        saveSelectedOptions();
        showSelectedOptions();
        runtime.reset();
        mStateTime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (mCurrentState) {
            case STATE_INITIAL:
                if (runtime.seconds() >= delayStartSeconds) {
                    newState(State.STATE_DELIVER_DUCK);
                }
                break;
            case STATE_DELIVER_DUCK:
                deliverDuck();
                newState(State.STATE_DELIVER_FREIGHT);
                break;
            case STATE_DELIVER_FREIGHT:
                deliverFreight();
                newState(State.STATE_PARK_IN_STORAGE);
                break;
            case STATE_PARK_IN_STORAGE:
                parkInStorage();
                newState(State.STATE_PARK_IN_WAREHOUSE);
                break;
            case STATE_PARK_IN_WAREHOUSE:
                parkInWarehouse();
                newState(State.STATE_STOP);
                break;
            case STATE_STOP:
                leftFront.setVelocity(0);
                rightFront.setVelocity(0);
                leftRear.setVelocity(0);
                rightRear.setVelocity(0);
            default:
                newState(State.STATE_STOP);
        }
        updateTelemetry();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //Make sure motors are stopped.
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rotor.setVelocity(0);
    }

    private void saveSelectedOptions() {
        // Save the driver selections for use in your autonomous strategy.
        alliance = autoConfig.getAlliance();
        startPosition = autoConfig.getStartPosition();
        parkLocation = autoConfig.getParklocation();
        deliverDuck = autoConfig.getDeliverDuck();
        deliverFreight = autoConfig.getDeliverFreight();
        delayStartSeconds = autoConfig.DelayStartSeconds();
    }

    private void showSelectedOptions() {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start position", startPosition);
        telemetry.addData("Park Location", parkLocation);
        telemetry.addData("Deliver Duck", deliverDuck);
        telemetry.addData("Deliver Freight", deliverFreight);
        telemetry.addData("Delay Start", delayStartSeconds);
        telemetry.addLine("----------------------");
    }

    private void updateTelemetry() {
        // Show the elapsed game time and wheel power.
        telemetry.addData("Run Time:", String.format("%4.0f ", runtime.time()));
        telemetry.addData("State Time:", String.format("%4.0f ", mStateTime.time()) + mCurrentState.toString());
        telemetry.addData("Motor Velocity", "left (%.2f), right (%.2f)", leftFront.getVelocity(), rightFront.getVelocity());
        telemetry.addData("Motor Position", "left (%d), right (%d)", leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
        telemetry.addData("Heading", imu.getAngularOrientation().firstAngle);
        telemetry.update();
    }

    private String TurnDirection(AutonomousConfiguration.AllianceColor alliance) {
        String result;
        if (alliance == AutonomousConfiguration.AllianceColor.Red) {
            result = "Left";
        } else {
            result = "Right";
        }
        return result;
    }

    //--------------------------------------------------------------------------
    //  Transition to a new state.
    //--------------------------------------------------------------------------
    private void newState(State newState) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Reset the state time, and then change to next state.
        mStateTime.reset();
        mCurrentState = newState;
    }

    private boolean ParkInStorage() {
        if (parkLocation != AutonomousConfiguration.ParkLocation.StorageUnit) {
            return false;
        }
        boolean startInFront = (startPosition == AutonomousConfiguration.StartPosition.Front);
        double storageDistance = (startPosition == AutonomousConfiguration.StartPosition.Front) ? 25 : 48;

        gyroDrive(DRIVE_SPEED, 2, 0);
        if (startInFront) {
            gyroDrive(DRIVE_SPEED, 24, 0);
            gyroTurn(.7, getAllianceTurnAngle(90));
            gyroDrive(DRIVE_SPEED, storageDistance, getAllianceTurnAngle(90));
        } else {
            gyroTurn(.7, getAllianceTurnAngle(90));
            gyroDrive(DRIVE_SPEED, storageDistance, getAllianceTurnAngle(90));
            gyroTurn(.7, 0);
            gyroDrive(DRIVE_SPEED, 24, 0);
            gyroTurn(.7, getAllianceTurnAngle(90));
            gyroDrive(DRIVE_SPEED, 24, getAllianceTurnAngle(90));
        }

        gyroStop();
        return true;
    }

    private boolean ParkInWarehouse() {
        if (parkLocation != AutonomousConfiguration.ParkLocation.WarehouseBack
                && parkLocation != AutonomousConfiguration.ParkLocation.WarehouseFront) {
            return false;
        }

        double wareHouseDistance = (startPosition == AutonomousConfiguration.StartPosition.Front) ? 75 : 27;

        gyroTurn(.7, getAllianceTurnAngle(-90));

        gyroDrive(DRIVE_SPEED, wareHouseDistance, getAllianceTurnAngle(-90));

        //
        if (parkLocation == AutonomousConfiguration.ParkLocation.WarehouseBack) {
            gyroTurn(.7, getAllianceTurnAngle(0));
            gyroDrive(DRIVE_SPEED, 21, getAllianceTurnAngle(0));
        }

        gyroStop();
        return true;
    }

    private boolean DeliverDuck() {
        if (deliverDuck == AutonomousConfiguration.DeliverDuck.No) {
            return false;
        }

        double duckDistance = (startPosition == AutonomousConfiguration.StartPosition.Front) ? -20 : -67;
        gyroDrive(DRIVE_SPEED, 2, getAllianceTurnAngle(0));
        gyroTurn(.5, getAllianceTurnAngle(-90));
        gyroDrive(DRIVE_SPEED, duckDistance, getAllianceTurnAngle(-90));

        double turnAngle = getAllianceTurnAngle(-45);
        //Adjust for rotor offset in blue.
        if (alliance == AutonomousConfiguration.AllianceColor.Blue) {
            turnAngle += 25;
        }

        gyroTurn(.5, turnAngle);
        gyroDrive(DRIVE_SPEED, -2.5, getAllianceTurnAngle(0));

        runRotor(true);
        ElapsedTime duckTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while (duckTime.seconds() <= 2) {
//            telemetry.update();
        }

        runRotor(false);
        // Go back to start
        gyroDrive(DRIVE_SPEED, 1, getAllianceTurnAngle(0));
        gyroTurn(.5, getAllianceTurnAngle(-90));
        gyroDrive(DRIVE_SPEED, -duckDistance, getAllianceTurnAngle(-90));
        gyroTurn(.5, getAllianceTurnAngle(0));
        gyroDrive(DRIVE_SPEED, -4, getAllianceTurnAngle(0));

        gyroStop();
        return true;
    }

    private boolean DeliverFreight() {
        if (deliverFreight == AutonomousConfiguration.DeliverFreight.No) {
            return false;
        }

        gyroDrive(DRIVE_SPEED, 2, 0);
        double turnAngle = getAllianceTurnAngle(-35);
        turnAngle = (startPosition == AutonomousConfiguration.StartPosition.Front) ? turnAngle : -turnAngle;
        gyroTurn(.5, turnAngle);

        gyroDrive(DRIVE_SPEED, 24, turnAngle);
        moveArm(8);
        openHand(true);
        ElapsedTime handPause = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (handPause.milliseconds() <= 500) {
        }

        moveArm(-8);
        //Go back to where we started
        gyroDrive(DRIVE_SPEED, -24, turnAngle);
        gyroTurn(.5, getAllianceTurnAngle(0));
        gyroDrive(DRIVE_SPEED, -2, getAllianceTurnAngle(0));
        gyroStop();
        return true;
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double velocity;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;


        // Determine new target position, and pass to motor controller
        moveCounts = getInchesToTicks(distance);
        newLeftTarget = leftFront.getCurrentPosition() + moveCounts;
        newRightTarget = rightFront.getCurrentPosition() + moveCounts;

        // Set Target and Turn On RUN_TO_POSITION
        leftFront.setTargetPosition(newLeftTarget);
        rightFront.setTargetPosition(newRightTarget);
        leftRear.setTargetPosition(newLeftTarget);
        rightRear.setTargetPosition(newRightTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        velocity = getVelocityFromPower(speed);
        leftFront.setVelocity(velocity);
        rightFront.setVelocity(velocity);
        leftRear.setVelocity(velocity);
        rightRear.setVelocity(velocity);

        // All motors are running.
        while (leftFront.isBusy()
                && rightFront.isBusy()
                && leftRear.isBusy()
                && rightRear.isBusy()) {

            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0) {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            // Adjust for setVelocity
            leftSpeed *= getVelocityFromPower(leftSpeed);
            rightSpeed += getVelocityFromPower(rightSpeed);

            leftFront.setVelocity(leftSpeed);
            rightFront.setVelocity(rightSpeed);
            leftRear.setVelocity(leftSpeed);
            rightRear.setVelocity(rightSpeed);
            updateTelemetry();
        }

        // Stop all motion;
        leftFront.setVelocity(0);
        rightFront.setVelocity(0);
        leftRear.setVelocity(0);
        rightRear.setVelocity(0);

        // Turn off RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are not on heading.
        while (!onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            updateTelemetry();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while ((holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            updateTelemetry();
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        //Adjust for setVelocity
        leftSpeed = getVelocityFromPower(leftSpeed);
        rightSpeed = getVelocityFromPower(rightSpeed);

        // Send desired speeds to motors.
        leftFront.setVelocity(leftSpeed);
        rightFront.setVelocity(rightSpeed);
        leftRear.setVelocity(leftSpeed);
        rightRear.setVelocity(rightSpeed);

        // Display it for the driver.
//        telemetry.addData("Target", "%5.2f", angle);
//        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
//        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation().firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void gyroStop() {
        leftFront.setVelocity(0);
        rightFront.setVelocity(0);
        leftRear.setVelocity(0);
        rightRear.setVelocity(0);
    }

    // Adjust the angle for red or blue alliance.
    private double getAllianceTurnAngle(double angle) {
        return (alliance == AutonomousConfiguration.AllianceColor.Red) ? angle : -angle;
    }

    private void runRotor(boolean isOn) {
        double rotorDirection = alliance == AutonomousConfiguration.AllianceColor.Red ? ROTOR_SPEED : -ROTOR_SPEED;
        rotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (isOn) {
            rotor.setVelocity(rotorDirection);
        } else {
            rotor.setVelocity(0);
        }
    }

    private void openHand(boolean open) {
        handServo.setPosition(open ? 0 : 1);
    }

    private void moveArm(double distance) {
        arm.setTargetPosition(getInchesToTicks(distance));
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(1200);
        ElapsedTime moveTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while (arm.isBusy() && moveTime.seconds() <= 2) {
        }

        arm.setVelocity(0);
    }

    private double getTicksPerSecond() {
        return (leftFront.getMotorType().getMaxRPM() / 60) * leftFront.getMotorType().getTicksPerRev();
    }

    private double getVelocityFromPower(double power) {
        return getTicksPerSecond() * power;
    }

    private int getInchesToTicks(double inches) {
        double circumference = WHEEL_DIAMETER * Math.PI;
        double rotations = inches / circumference;
        return (int) (rotations * leftFront.getMotorType().getTicksPerRev());
    }

    // States for navigation.
    private enum State {
        STATE_INITIAL,
        STATE_PARK_IN_STORAGE,
        STATE_PARK_IN_WAREHOUSE,
        STATE_DELIVER_DUCK,
        STATE_DELIVER_FREIGHT,
        STATE_DRIVE_FORWARD,
        STATE_TURN_90,
        STATE_DRIVE_TO_WALL,
        STATE_BACKUP,
        STATE_STOP
    }
}

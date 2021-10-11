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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 */

@Autonomous(name = "Example: Auto Config Menu", group = "Example")
//@Disabled
public class AutoConfigMenu extends OpMode {
    // Declare OpMode members.
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower = 0;
    double rightPower = 0;
    private AutonomousConfiguration autoConfig;
    private boolean optionsSelected = false;
    private AutonomousConfiguration.AllianceColor alliance;
    private AutonomousConfiguration.StartPosition startPosition;
    private AutonomousConfiguration.ParkLocation parkLocation;
    private AutonomousConfiguration.DeliverDuck deliverDuck;
    private AutonomousConfiguration.DeliverFreight deliverFreight;
    private final DcMotor leftDrive = null;
    private final DcMotor rightDrive = null;

    // States for navigation.
    private enum State {
        STATE_INITIAL,
        STATE_DRIVE_FORWARD,
        STATE_TURN_90,
        STATE_DRIVE_TO_WALL,
        STATE_BACKUP,
        STATE_STOP
    }

    // Loop cycle time stats variables
    private ElapsedTime mStateTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);  // Time into current state
    private State mCurrentState;    // Current State Machine State.

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
//        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
//        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        autoConfig = new AutonomousConfiguration(gamepad1, telemetry);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
//        leftDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        mCurrentState = State.STATE_INITIAL;
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
            ShowSelectedOptions();
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (mCurrentState) {
            case STATE_INITIAL:
                newState(State.STATE_DRIVE_FORWARD);
                runtime.reset();
                break;

            // Step 1:  Drive forward.
            case STATE_DRIVE_FORWARD:
//        robot.leftDrive.setPower(FORWARD_SPEED);
//        robot.rightDrive.setPower(FORWARD_SPEED);
                if ((runtime.seconds() >= 3.0)) {
                    newState(State.STATE_TURN_90);
                    runtime.reset();
                }

                break;

            // Step 2:  Spin left/right.
            case STATE_TURN_90:
                if (alliance == AutonomousConfiguration.AllianceColor.Red) {
//            robot.leftDrive.setPower(-TURN_SPEED);
//            robot.rightDrive.setPower(+TURN_SPEED);
                } else {
//            robot.leftDrive.setPower(TURN_SPEED);
//            robot.rightDrive.setPower(-TURN_SPEED);
                }

                if ((runtime.seconds() >= 4)) {
                    newState(State.STATE_DRIVE_TO_WALL);
                    runtime.reset();
                }

                break;

            // Step 3:  Drive to wall
            case STATE_DRIVE_TO_WALL:
//        robot.leftDrive.setPower(FORWARD_SPEED);
//        robot.rightDrive.setPower(FORWARD_SPEED);
                if ((runtime.seconds() >= 3.0)) {
                    newState(State.STATE_BACKUP);
                    runtime.reset();
                }

                break;

            // Step 4:  Backup.
            case STATE_BACKUP:
//        robot.leftDrive.setPower(-FORWARD_SPEED);
//        robot.rightDrive.setPower(-FORWARD_SPEED);
                if ((runtime.seconds() >= 0.75)) {
                    newState(State.STATE_STOP);
                    runtime.reset();
                }

                break;
            case STATE_STOP:
                telemetry.addData("Path", "Complete");
//        robot.leftDrive.setPower(0);
//        robot.rightDrive.setPower(0);
                break;
            // This should never happen.
            default:
                mCurrentState = State.STATE_INITIAL;
        }
        // Show the elapsed game time and wheel power.
        telemetry.addData("Run Time:", String.format("%4.1f ", runtime.time()));
        telemetry.addData("State Time:", String.format("%4.1f ", mStateTime.time()) + mCurrentState.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Turn:", TurnDirection(alliance));
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void ShowSelectedOptions() {
        // Save the driver selections for use in your autonomous strategy.
        alliance = autoConfig.getAlliance();
        startPosition = autoConfig.getStartPosition();
        parkLocation = autoConfig.getParklocation();
        deliverDuck = autoConfig.getDeliverDuck();
        deliverFreight = autoConfig.getDeliverFreight();

        telemetry.clear();
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start position", startPosition);
        telemetry.addData("Park Location", parkLocation);
        telemetry.addData("Deliver Duck", deliverDuck);
        telemetry.addData("Deliver Freight", deliverFreight);
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
        // Reset the state time, and then change to next state.
        mStateTime.reset();
        mCurrentState = newState;
    }

    private void ParkInAllianceStorage() {

    }
}

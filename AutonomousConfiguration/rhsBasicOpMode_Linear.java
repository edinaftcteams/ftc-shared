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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
    This is a basic linear opMode example for testing the AutonomousConfiguration class.
    The AutonomousConfiguration class can also be used with an iterative opMode.
 */
@Autonomous(name = "Menu Test", group = "Linear")
//@Disabled
public class rhsBasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    // This the configuration class used to get the driver's selections.
    private AutonomousConfiguration autoConfig;
    // The properties are available after the
    // call to the ShowMenu method of the AutonomousConfiguration class.
    private AutonomousConfiguration.AllianceColor alliance;
    private AutonomousConfiguration.StartPosition startPosition;
    private AutonomousConfiguration.NavigationLane navigationLane;
    private AutonomousConfiguration.Deliver deliver;
    private AutonomousConfiguration.Reposition reposition;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        GetAutonomousConfigurationOptions();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Path", "Driving Autonomous");
            telemetry.update();

            /*
             Replace this sample code with your path code using the configuration
             options from the menu selection to control the logic.
             */
            if (startPosition == AutonomousConfiguration.StartPosition.Back) {
                Navigate();
            }

            if (startPosition == AutonomousConfiguration.StartPosition.Front) {
                Reposition();
            }

        }
    }

    private void Navigate() {
        double leftPower;
        double rightPower;
        double runSeconds;

        // Move away from wall
        leftPower = .3;
        rightPower = .3;
        runSeconds = navigationLane == AutonomousConfiguration.NavigationLane.Inside ? 1 : 2.5;
        RunMotors(leftPower, rightPower, runSeconds);

        // Turn 90 degrees
        if (alliance == AutonomousConfiguration.AllianceColor.Red) {
            rightPower = 0;
        } else {
            leftPower = 0;
        }

        runSeconds = 1;
        RunMotors(leftPower, rightPower, runSeconds);

        // Drive under sky bridge
        leftPower = .3;
        rightPower = .3;
        runSeconds = 3.5;
        RunMotors(leftPower, rightPower, runSeconds);
    }

    private void Reposition() {
        double leftPower;
        double rightPower;
        double runSeconds;

        // Move away from wall
        leftPower = .3;
        rightPower = .3;
        runSeconds = 1;
        RunMotors(leftPower, rightPower, runSeconds);

        // Turn 90 degrees
        if (alliance == AutonomousConfiguration.AllianceColor.Red) {
            rightPower = 0;
        } else {
            leftPower = 0;
        }

        runSeconds = 1;
        RunMotors(leftPower, rightPower, runSeconds);

        // Drive towards foundation
        leftPower = .3;
        rightPower = .3;
        runSeconds = 4;
        RunMotors(leftPower, rightPower, runSeconds);

        // Turn 90 degrees towards foundation
        if (alliance == AutonomousConfiguration.AllianceColor.Red) {
            leftPower = 0;
        } else {
            rightPower = 0;
        }

        runSeconds = 1;
        RunMotors(leftPower, rightPower, runSeconds);

        // Drive to foundation
        leftPower = .3;
        rightPower = .3;
        runSeconds = 2.5;
        RunMotors(leftPower, rightPower, runSeconds);

        // Drop your dragging device here.
        // Drag to the building site
        leftPower = -.3;
        rightPower = -.3;
        runSeconds = 3.5;
        RunMotors(leftPower, rightPower, runSeconds);

        // Straf back to the sky bridge if you have mecanum wheels!
    }

    private void RunMotors(double leftPower, double rightPower, double runSeconds) {
        runtime.reset();
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        while (runtime.seconds() < runSeconds) {
            sleep(10);
        }

        StopMotors();
    }

    private void StopMotors() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private void GetAutonomousConfigurationOptions() {
        // Get configuration selections from the driver using gamepad1.
        autoConfig = new AutonomousConfiguration(gamepad1, telemetry);
        autoConfig.ShowMenu();

        // Save the driver selections for use in your autonomous strategy.
        alliance = autoConfig.getAlliance();
        startPosition = autoConfig.getStartPosition();
        navigationLane = autoConfig.getNavigationLane();
        deliver = autoConfig.getDeliver();
        reposition = autoConfig.getReposition();

        telemetry.clear();
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start position", startPosition);
        telemetry.addData("Navigation Lane", navigationLane);
        telemetry.addData("Deliver", deliver);
        telemetry.addData("Reposition", reposition);
        telemetry.update();
    }
}

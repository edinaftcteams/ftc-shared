package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Ron on 11/16/2016.
 * This class provides configuration for the autonomous opmode.
 */

public class AutonomousConfiguration {
    public AutonomousConfiguration(Gamepad gamepad, Telemetry telemetry1) {
        this.gamepad1 = gamepad;
        this.telemetry = telemetry1;
        alliance = AllianceColor.None;
        // Default selections if driver does not select any.
        startPosition = StartPosition.Center;
        parkLocation = ParkLocation.Ramp;
        pressBeacon = false;
    }

    private AllianceColor alliance;
    private StartPosition startPosition;
    private ParkLocation parkLocation;
    private boolean pressBeacon;
    private boolean launchParticle;
    private Gamepad gamepad1 = null;

    // Seconds to delay before starting opmode.
    private int startDelay = 0;

    // Where do we place the robot
    public enum StartPosition {
        Center,
        Left;

        public StartPosition getNext() {
            return values()[(ordinal() + 1) % values().length];
        }

    }

    // Where do we park
    public enum ParkLocation {
        Center,
        Ramp;

        public ParkLocation getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    public enum AllianceColor {
        None,
        Red,
        Blue
    }

    private Telemetry telemetry = null;

    public int getStartDelay() {
        return startDelay;
    }

    public AllianceColor getAlliance() {
        return alliance;
    }

    public StartPosition getStartPosition() {
        return startPosition;
    }

    public ParkLocation getParkLocation() {
        return parkLocation;
    }

    public boolean getPressBeacon() {
        return pressBeacon;
    }

    public boolean getLaunchParticle() {
        return launchParticle;
    }

    public void ShowMenu() {
        boolean bCurrStatePadLeft = false;
        boolean bPrevStatePadLeft = false;
        boolean bCurrStatePadRight = false;
        boolean bPrevStatePadRight = false;

        do {
            if (gamepad1.x) {
                alliance = AllianceColor.Blue;
            }

            if (gamepad1.b) {
                alliance = AllianceColor.Red;
            }

            bCurrStatePadLeft = gamepad1.dpad_left;
            if ((bCurrStatePadLeft && (bCurrStatePadLeft != bPrevStatePadLeft))
                    && (startDelay > 0)) {
                startDelay--;
            }

            bPrevStatePadLeft = bCurrStatePadLeft;

            bCurrStatePadRight = gamepad1.dpad_right;
            if ((bCurrStatePadRight && (bCurrStatePadRight != bPrevStatePadRight))
                    && (startDelay < 20)) {
                startDelay++;
            }

            bPrevStatePadRight = bCurrStatePadRight;

            if (gamepad1.y) {
                startPosition = startPosition.getNext();
            }

            if (gamepad1.a) {
                parkLocation = parkLocation.getNext();
            }

            if (gamepad1.right_bumper) {
                pressBeacon = !pressBeacon;
            }

            if (gamepad1.left_bumper) {
                launchParticle = !launchParticle;
            }

            // Only allow loop exit if alliance has been selected.
            if (gamepad1.start && alliance != AllianceColor.None) {
                break;
            }

            telemetry.addData("Menu", "x = Blue, b = Red,");
            telemetry.addData("", "dpad left(decrease), right(increase) start delay");
            telemetry.addData("", "y to cycle start position");
            telemetry.addData("", "a to cycle park location");
            telemetry.addData("", "right bumper to cycle press beacon");
            telemetry.addData("", "left bumper to cycle launch particle");
            telemetry.addData("Finished", "Press gamepad Start");
            telemetry.addData("Selected", "Alliance %s", alliance);
            telemetry.addData("", "Start delay %d", startDelay);
            telemetry.addData("", "Start position %s", startPosition);
            telemetry.addData("", "Park location %s", parkLocation);
            telemetry.addData("", "Press the beacon %s", pressBeacon);
            telemetry.addData("", "Launch particle %s", launchParticle);
            telemetry.update();
        } while (true);
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

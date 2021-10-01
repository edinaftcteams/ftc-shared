package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Ron on 11/16/2016.
 * Modified: 10/1/2021
 * <p>
 * This class provides configuration for an autonomous opMode.
 * Most games benefit from autonomous opModes that can implement
 * different behavior based on an alliance strategy agreed upon
 * for a specific match.
 * </p>
 * <p>
 * Creating multiple opModes to meet this requirement results in duplicate
 * code and an environment that makes it too easy for a driver to
 * choose the wrong opMode "in the heat of battle."
 * </p>
 * <p>
 * This class is a way to solve these problems.
 * </p>
 */

public class AutonomousConfiguration {
    /*
     Pass in gamepad and telemetry from your opMode when creating this object.
     */
    public AutonomousConfiguration(Gamepad gamepad, Telemetry telemetry1) {
        this.gamePad1 = gamepad;
        this.telemetry = telemetry1;
        // Default selections if driver does not select anything.
        alliance = AllianceColor.None;
        startPosition = StartPosition.None;
        parklocation = ParkLocation.None;
        deliverFreight = DeliverFreight.No;
        deliverDuck = DeliverDuck.No;
    }

    private AllianceColor alliance;
    private StartPosition startPosition;
    private ParkLocation parklocation;
    private DeliverDuck deliverDuck;
    private DeliverFreight deliverFreight;
    private Gamepad gamePad1;
    private Telemetry telemetry;

    public enum AllianceColor {
        None,
        Red,
        Blue
    }

    /*
        Where do we start the robot
        Back is towards the warehouse.
        Front if towards the audience.
     */
    public enum StartPosition {
        None,
        Back,
        Front;

        public StartPosition getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    /*
        Where do we park. Default is do not park.
     */
    public enum ParkLocation {
        None,
        WarehouseFront,
        WarehouseBack,
        StorageUnit;

        public ParkLocation getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    /*
        Yes means deliver the duck from the carousel.
        Default is No.
     */
    public enum DeliverDuck {
        No,
        Yes;

        public DeliverDuck getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    /*
        Yes means deliver freight to the shipping hub.
        Default is No.
     */
    public enum DeliverFreight {
        No,
        Yes;

        public DeliverFreight getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    public AllianceColor getAlliance() {
        return alliance;
    }

    public StartPosition getStartPosition() {
        return startPosition;
    }

    public ParkLocation getParklocation() {
        return parklocation;
    }

    public DeliverDuck getDeliverDuck() {
        return deliverDuck;
    }

    public DeliverFreight getDeliverFreight() {
        return deliverFreight;
    }

    // Call this from your opMode to show the menu for selection.
    public void ShowMenu() {
        ElapsedTime runTime = new ElapsedTime();
        telemetry.setAutoClear(false);
        Telemetry.Item teleAlliance = telemetry.addData("X = Blue, B = Red", getAlliance());
        Telemetry.Item teleStartPosition = telemetry.addData("D-pad left/right, select start position", getStartPosition());
        Telemetry.Item teleParkLocation = telemetry.addData("D-pad up to cycle park location", getParklocation());
        Telemetry.Item teleDeliverduck = telemetry.addData("D-pad down to cycle deliver duck", getDeliverDuck());
        Telemetry.Item teleDeliverFreight = telemetry.addData("Left Bumper to cycle deliver freight", getDeliverFreight());
        telemetry.addData("Finished", "Press game pad Start");

        // Loop while driver makes selections.
        do {
            if (gamePad1.x) {
                alliance = AllianceColor.Blue;
            }

            if (gamePad1.b) {
                alliance = AllianceColor.Red;
            }

            teleAlliance.setValue(alliance);

            if (gamePad1.dpad_left) {
                startPosition = StartPosition.Back;
            }

            if (gamePad1.dpad_right) {
                startPosition = StartPosition.Front;
            }

            teleStartPosition.setValue(startPosition);

            if (gamePad1.dpad_up) {
                parklocation = parklocation.getNext();
            }

            teleParkLocation.setValue(parklocation);

            if (gamePad1.dpad_down) {
                deliverDuck = deliverDuck.getNext();
            }

            teleDeliverduck.setValue(deliverDuck);

            if (gamePad1.left_bumper) {
                deliverFreight = deliverFreight.getNext();
            }

            teleDeliverFreight.setValue(deliverFreight);

            telemetry.update();

            // If there is no gamepad timeout for debugging.
            if (gamePad1.id == -1) {
                // The timer is for debugging, remove it when you have a gamepad connected.
                if (runTime.seconds() > 3) {
                    break;
                }
            } else {
                // Only allow loop exit if alliance has been selected.
                if (gamePad1.start && alliance != AllianceColor.None) {
                    break;
                }
            }
        } while (true);
    }
}

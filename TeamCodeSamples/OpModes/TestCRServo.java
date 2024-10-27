package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test CR Servo", group = "test")
//@Disabled
public class TestCRServo extends OpMode {
    static final double INCREMENT = 0.1;     // amount to change servo speed each button press.
    private CRServo servo;
    private double position = 0;
    private GamepadEx gamepad;

    @Override
    public void init() {
        // Make the name match your config file and robot.
        servo = new CRServo(hardwareMap, "crservo1");
        gamepad = new GamepadEx(gamepad1);
        showTelemetry();
        telemetry.update();
    }

    @Override
    public void start() {
        servo.setRunMode(Motor.RunMode.RawPower);
        servo.set(position);
    }

    @Override
    public void loop() {
        gamepad.readButtons();
        if (gamepad.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
            position = -.8;
        }

        if (gamepad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
            position = .8;
        }

        if (gamepad.wasJustReleased(GamepadKeys.Button.Y)) {
            position = 0;
        }

        if (gamepad.wasJustReleased(GamepadKeys.Button.DPAD_UP) && position < .8) {
            position += INCREMENT;
        }

        if (gamepad.wasJustReleased(GamepadKeys.Button.DPAD_DOWN) && position > -.8) {
            position -= INCREMENT;
        }

        servo.set(position);
        showTelemetry();
    }

    private void showTelemetry() {
        telemetry.addLine("Left bumper = -.8");
        telemetry.addLine("Right bumper = .8");
        telemetry.addLine("Y = .5 (stop)");
        telemetry.addLine("Dpad up: Increase position");
        telemetry.addLine("Dpad down: Decrease position");
        telemetry.addData("Position", servo.get());
    }
}

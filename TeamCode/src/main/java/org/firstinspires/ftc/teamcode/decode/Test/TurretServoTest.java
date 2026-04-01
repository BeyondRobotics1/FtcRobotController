package org.firstinspires.ftc.teamcode.decode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.decode.Subsystems.Trigger;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Turret;

@TeleOp(name = "Turret Servo Test", group = "Decode Test")
public class TurretServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("gamepad1.a", "Red Obelisk position." );
        telemetry.addData("gamepad1.b", "Blue Obelisk position." );
        telemetry.addData("gamepad1.x", "Center position." );
        telemetry.addData("gamepad1.dpad_left", "Left position." );
        telemetry.addData("gamepad1.dpad_right", "Right position." );
        telemetry.addData("gamepad1.left_trigger", "rotating." );
        telemetry.update();

        Servo turretLeft, turretRight;

        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        turretRight = hardwareMap.get(Servo.class, "turretRight");

//        turretLeft.setDirection(Servo.Direction.REVERSE);
//        turretRight.setDirection(Servo.Direction.REVERSE);

        turretLeft.setPosition(Turret.servoPositionMiddle);
        turretRight.setPosition(Turret.servoPositionMiddle);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            if(gamepad1.a) {
                turretLeft.setPosition(Turret.servoPositionObeliskDetectionRedAlliance);
                turretRight.setPosition(Turret.servoPositionObeliskDetectionRedAlliance);
            }
            else if (gamepad1.b) {
                turretLeft.setPosition(Turret.servoPositionObeliskDetectionBlueAlliance);
                turretRight.setPosition(Turret.servoPositionObeliskDetectionBlueAlliance);
            }
            else if (gamepad1.dpad_left) {
                turretLeft.setPosition(Turret.servoPositionLeft);
                turretRight.setPosition(Turret.servoPositionLeft);
            }
            else if (gamepad1.dpad_right) {
                turretLeft.setPosition(Turret.servoPositionRight);
                turretRight.setPosition(Turret.servoPositionRight);
            }
            else if (gamepad1.x) {
                turretLeft.setPosition(Turret.servoPositionMiddle);
                turretRight.setPosition(Turret.servoPositionMiddle);
            }
            else {

                double pivotPosition = Math.abs(gamepad1.left_trigger);

                turretLeft.setPosition(pivotPosition);
                turretRight.setPosition(pivotPosition);
            }

            telemetry.addData("Servo position", turretLeft.getPosition());

            telemetry.update();
        }

    }
}

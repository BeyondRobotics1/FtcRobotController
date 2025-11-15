package org.firstinspires.ftc.teamcode.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Concept: Magnetic Switch test", group = "Concept")
@Disabled
public class MagneticSwitchTest extends LinearOpMode {


    @Override
    public void runOpMode() {
        TouchSensor rev = hardwareMap.get(TouchSensor.class, "revm");
        TouchSensor swyft = hardwareMap.get(TouchSensor.class, "swm");
        DcMotor motor1 = hardwareMap.get(DcMotor.class, "motor1");

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            // If the Magnetic Limit Switch is pressed, stop the motor
            if (rev.isPressed()) {
                motor1.setPower(0);
            } else { // Otherwise, run the motor
                motor1.setPower(0.3);
            }

            telemetry.addData("Arm Motor Power:", motor1.getPower());
            telemetry.update();
        }
    }
}

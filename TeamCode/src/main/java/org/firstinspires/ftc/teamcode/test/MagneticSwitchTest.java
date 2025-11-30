package org.firstinspires.ftc.teamcode.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Concept: Magnetic Switch test", group = "Concept")
public class MagneticSwitchTest extends LinearOpMode {


    @Override
    public void runOpMode() {
        TouchSensor rev = hardwareMap.get(TouchSensor.class, "magnet");
        Servo indexer = hardwareMap.get(Servo.class, "indexer");

        waitForStart();
        indexer.setPosition(0.5);
        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            if(gamepad1.bWasPressed()){
                indexer.setPosition(0.9);

            }else if(rev.isPressed()&& gamepad1.bWasReleased()){
                indexer.setPosition(0.5);
            }
            if(gamepad1.aWasPressed()){
                indexer.setPosition(0.1);


            }else if(rev.isPressed() && gamepad1.aWasReleased()){
                indexer.setPosition(0.5);
            }



            // If the Magnetic Limit Switch is pressed, stop the motor


            telemetry.addData("Arm Motor Power:", indexer.getPosition());
            telemetry.addData("detected?", rev.isPressed());
            telemetry.update();
        }
    }
}

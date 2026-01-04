package org.firstinspires.ftc.teamcode.decode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Clutch Slide Servo test", group = "Decode Test")
public class ClutchSlideServoTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        Servo servo = null;// = hardwareMap.get(Servo.class, "clutchRight");
        String servoName = "";

        telemetry.addData("gamepad1.a", "Clutch Left Servo." );
        telemetry.addData("gamepad1.b", "Clutch Right Servo." );
        telemetry.addData("gamepad1.x", "Slide Left Servo." );
        telemetry.addData("gamepad1.y", "Slide Right Servo." );
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive())
        {
            if(servo == null) {
                if (gamepad1.a) {
                    servo = hardwareMap.get(Servo.class, "clutchLeft");
                    servo.setDirection(Servo.Direction.REVERSE);
                    servoName = "clutchLeft";
                }
                else if (gamepad1.b) {
                    servo = hardwareMap.get(Servo.class, "clutchRight");
                    //servo.setDirection(Servo.Direction.REVERSE);
                    servoName = "clutchRight";
                }
                else if (gamepad1.x) {
                    servo = hardwareMap.get(Servo.class, "slideLeft");
                    servo.setDirection(Servo.Direction.REVERSE);
                    servoName = "slideLeft";
                }
                else if (gamepad1.y) {
                    servo = hardwareMap.get(Servo.class, "slideRight");
                    servoName = "slideRight";
                }
            }

            if(servo != null) {

                //use gamepad1 left trigger to set servo positions dynamically
                double pivotPosition = gamepad1.left_trigger;

                telemetry.addData(servoName+ " position", Math.abs(pivotPosition));
                servo.setPosition(pivotPosition);
            }

            telemetry.update();

        }

    }
}

package org.firstinspires.ftc.teamcode.tutorial.Michael;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp (name = "Hello World", group = "Michael")
@Disabled
public class HelloWorld extends OpMode {
    /**
     * This is called when the driver pressed init
     */
    @Override
    public void init() {
        // this sends to the driver station
        /*telemetry.addData("Hello", "World");

        int teamNumber = 21380;
        double motorSpeed = 0.5;
        boolean touchSensorPressed = false;

        telemetry.addData("Team Number", teamNumber);
        telemetry.addData("Motor Speed", motorSpeed);
        telemetry.addData("Touch Sensor", touchSensorPressed);

        String myName = "Michael";

        telemetry.addData("Hello", myName); */
    }

    double squareInputWithSign(double input){
        double output = input * input;
        if(input < 0){
            output = output * -1;
        }
        return output;
    }

    /**
     * This is called repeatedly while OpMode is playing
     */
    @Override
    public void loop(){
        double leftAmount = gamepad1.left_stick_x;
        double fwdAmount = -gamepad1.left_stick_y;

        telemetry.addData("Before X", leftAmount);
        telemetry.addData("Before Y", fwdAmount);

        leftAmount = squareInputWithSign(leftAmount);
        fwdAmount = squareInputWithSign(fwdAmount);

        telemetry.addData("Before X", leftAmount);
        telemetry.addData("Before Y", fwdAmount);

        /*telemetry.addData("Left stick x", gamepad1.left_stick_x);
        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        telemetry.addData("A button", gamepad1.a);

        double speedForward = -gamepad1.left_stick_y / 2.0;

        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        telemetry.addData("speed Forward", speedForward);

        if (gamepad1.left_stick_y < -0.5) {
            telemetry.addData("Left stick", " is negative and large");
        }
        else if (gamepad1.left_stick_y < 0 && gamepad1.left_stick_y > -0.5){
            telemetry.addData("Left stick", "is negative and small");
        }
        else if (gamepad1.left_stick_y > 0 && gamepad1.left_stick_y < 0.5){
            telemetry.addData("Left stick", "is positive and small");
        }
        else {
            telemetry.addData("Left stick", " is positive and large");
        }
        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        */

        if(gamepad1.left_bumper){
            requestOpModeStop();
        }

    }
}

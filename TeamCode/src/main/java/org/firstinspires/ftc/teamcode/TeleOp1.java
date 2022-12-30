package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "TeleOp1", group = "TeleOp")
public class TeleOp1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // per Q & A 191,
        //REV Digital LED Indicator (https://www.revrobotics.com/rev-31-2010/) is NOT legal :(
        /*
        DigitalChannel redLED = hardwareMap.get(DigitalChannel.class, "endgame_red");
        DigitalChannel greenLED = hardwareMap.get(DigitalChannel.class, "endgame_green");

        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setState(true);
        redLED.setState(false);*/

        ////our robot hardware
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        Slide slide = new Slide(hardwareMap, this);
        Turret turret = new Turret(hardwareMap, slide);
        Claw claw = new Claw(hardwareMap, this);

        //We use this timer to check the game time that has elapsed
        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;

        //restart the timer
        timer.reset();

        boolean previousBumperState   = gamepad2.right_bumper;
        while (opModeIsActive()) {

            boolean currentBumperState  = gamepad2.right_bumper;

            //hold right bumper to close the claw
            if (currentBumperState)
            {
                claw.close();
                sleep(100);
                if (claw.holdingCone() && previousBumperState != currentBumperState)
                    slide.moveTo(slide.getSlideHeightInches() + 5.5, 1);
            }
            else //release right bumper to open the claw
                claw.open();

            previousBumperState  = currentBumperState ;

            //Move slide to specific junction height
            //left bumper + a dpad key
            if(gamepad2.left_bumper) {
                if (gamepad2.dpad_down)
                    slide.moveToJunctionWithoutWaiting(0, 1);
                else if (gamepad2.dpad_left)
                    slide.moveToJunctionWithoutWaiting(1, 1);
                else if (gamepad2.dpad_up)
                    slide.moveToJunctionWithoutWaiting(2, 1);
                else if (gamepad2.dpad_right)
                    slide.moveToJunctionWithoutWaiting(3, 1);
            }
            else //otherwise uses left stick y
            {
                slide.setPower(-gamepad2.left_stick_y);

//                //use left stick y to set the power slide motors
//                double slidePower = -gamepad2.left_stick_y;
//                slide.setPower(slidePower);
//                telemetry.addData("Slide power", slidePower);
//                telemetry.addData("Low Limit Touch Sensor", slide.getTouchSensorState(true));
//                telemetry.addData("High Limit Touch Sensor", slide.getTouchSensorState(false));
            }

            //Using right stick x and y for turret position
            if (Math.abs(gamepad2.right_stick_y) > 0.8) { //front position
                turret.setPositionCheckSlideHeight(1);
            } else if (gamepad2.right_stick_x > 0.8) { //right position
                turret.setPositionCheckSlideHeight(2);
            } else if (gamepad2.right_stick_x < -0.8) { //left position
                turret.setPositionCheckSlideHeight(0);
            }

            //drive train
            driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.update();

//            // INDICATION OF ENDGAME START 5 SECOND LATER (WARNING) 100% WORKING
//            if(timer.time(TimeUnit.SECONDS) >= 85){
//                redLED.setState(true);
//                greenLED.setState(false);
//            }

            //sleep(50);
        }
    }
}

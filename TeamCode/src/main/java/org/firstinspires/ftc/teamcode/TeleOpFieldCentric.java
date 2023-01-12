package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Field Centric", group = "TeleOp")
@Disabled
public class TeleOpFieldCentric extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DigitalChannel redLED = hardwareMap.get(DigitalChannel.class, "endgame_red");
        DigitalChannel greenLED = hardwareMap.get(DigitalChannel.class, "endgame_green");

        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setState(true);
        redLED.setState(false);

        //drivetrain
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this, true);

        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap, this);

        //We use this timer to check the game time that has elapsed
        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;

        //restart the timer
        timer.reset();

        while (opModeIsActive()) {

            //hold right bumper to close the claw
            if (gamepad2.right_bumper)
                claw.close();
            else //release right bumper to open the claw
                claw.open();

            //Using right stick x and y for turret position
            if (Math.abs(gamepad2.right_stick_y) > 0.8) { //front position
                arm.setTurretPosition(1);
            } else if (gamepad2.right_stick_x > 0.8) { //right position
                arm.setTurretPosition(2);
            } else if (gamepad2.right_stick_x < -0.8) { //left position
                arm.setTurretPosition(0);
            }

            //use left stick y to set the power slide motors
            arm.setSlidePower(-gamepad2.left_stick_y);
            //telemetry.addData("Slide motor position", arm.getSlideMotorCurrentPosition());
            //telemetry.addData("Low Limit Touch Sensor", arm.getTouchSensorState(true));
            //telemetry.addData("High Limit Touch Sensor", arm.getTouchSensorState(false));

            double slideHeight = arm.getDistanceINCH();
            telemetry.addData("Slide height inches", slideHeight );
            //double leftBackDistance = driveTrain.getDistanceINCH();
            //telemetry.addData("Left back distance inches",leftBackDistance );

            //drive train
            driveTrain.setPower2(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            //telemetry.update();

            // INDICATION OF ENDGAME START 5 SECOND LATER (WARNING) 100% WORKING
            if(timer.time(TimeUnit.SECONDS) >= 85){
                redLED.setState(true);
                greenLED.setState(false);
            }

            //sleep(50);
        }
    }
}

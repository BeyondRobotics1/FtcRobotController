package org.firstinspires.ftc.teamcode.powerplay;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "IntoTheDeep1", group = "TeleOp")
//@Disabled
public class IntoThe1 extends LinearOpMode {

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
        telemetry.addLine("Initializing drive train");
        telemetry.update();
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this, false);
        //We use this timer to check the game time that has elapsed
        ElapsedTime timer = new ElapsedTime();

        //
        telemetry.addLine("Initialization done, wait for start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addLine("All ready!!!");
        telemetry.update();

        //restart the timer
        timer.reset();
        boolean previousBumperState   = gamepad2.right_bumper;

        while (opModeIsActive()) {
            if(gamepad1.left_bumper)
                driveTrain.setPower2(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            else
                driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }
}

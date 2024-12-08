package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@TeleOp(name = "DriveSimply", group = "TeleOp")

public class DriveSimply extends LinearOpMode {
    private Timer timer = new Timer();
    @Override
    public void runOpMode() throws InterruptedException {
        boolean fieldcentric = false;
        boolean swap = false;
        double head = 0;
        GoBildaPinpointDriverRR pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class,"pinpoint");

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
        SimpleDriveTrain driveTrain = new SimpleDriveTrain(hardwareMap, this, false);

        //We use this timer to check the game time that has elapsed

        //
        telemetry.addLine("Initialization done, wait for start");
        telemetry.update();

        waitForStart();
        timer.resetTimer();
        if (isStopRequested()) return;


        telemetry.addLine("All ready!!!");
        telemetry.update();

        //restart the timer
        while (opModeIsActive()) {
            telemetry.addData("heading", Math.toDegrees(pinpoint.getPosition().getHeading(AngleUnit.RADIANS))-head);
            telemetry.addData("fieldcentric:", fieldcentric);
            telemetry.update();
            pinpoint.update();
            if (gamepad1.left_bumper && timer.getElapsedTimeSeconds() > 1){
                fieldcentric = !fieldcentric;
                timer.resetTimer();
            }

            if (fieldcentric)
                driveTrain.setPower2(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, pinpoint.getPosition().getHeading(AngleUnit.RADIANS) - head);
            else
                driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetry.update();
            if (gamepad1.back){
                head = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
            }
        }

    }
}

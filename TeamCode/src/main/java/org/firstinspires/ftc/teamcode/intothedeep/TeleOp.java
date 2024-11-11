package org.firstinspires.ftc.teamcode.intothedeep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Into the deep", group = "TeleOp")

public class TeleOp extends LinearOpMode {

    enum SlideOp
    {
        GROUND,
        LOW,
        MEDIUM,
        HIGH,
        SCORE,
        MANUAL
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initializing drive train");
        telemetry.update();

        PinpointDrive driveTrain = new PinpointDrive(hardwareMap, new Pose2d(0,0, 0));

        Slide slide = new Slide(hardwareMap, this);

        Claw claw = new Claw(hardwareMap, this);

        waitForStart();

        SlideOp slideOp = SlideOp.MANUAL;
        boolean robotCentric = true;

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            slideOp = SlideOp.MANUAL;
            slide.setPower(-gamepad2.left_stick_y);

            slide.autoMoveToWithoutWaitingLoop();


            boolean currentBumperState  = gamepad2.right_bumper;

            //hold right bumper to close the claw
            if (currentBumperState)
                claw.close();
            else
                claw.open();


            //drive train
            if(gamepad1.left_bumper)
                driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            else
                driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }
}

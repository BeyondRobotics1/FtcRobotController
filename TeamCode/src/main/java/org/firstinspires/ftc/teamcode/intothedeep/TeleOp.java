package org.firstinspires.ftc.teamcode.intothedeep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Into the deep", group = "TeleOp")

public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        //telemetry.addLine("Initializing drive train");
        //telemetry.update();
        //PinpointDrive driveTrain = new PinpointDrive(hardwareMap, new Pose2d(0,0, 0));

        telemetry.addLine("Initializing slide");
        telemetry.update();
        Slide slide = new Slide(hardwareMap, this);
        slide.runWithEncoder();


        telemetry.addLine("Initializing claw");
        telemetry.update();
        Claw claw = new Claw(hardwareMap, this);



        waitForStart();

        //slide is manually controlled
        Slide.SlideTargetPosition slideOp = Slide.SlideTargetPosition.MANUAL;
        Arm.ArmTargetAngle armOp = Arm.ArmTargetAngle.MANUAL;

        //arm is manually controlled

        boolean robotCentric = true;

        boolean sampleMode = true;

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            if(gamepad1.left_bumper)
            {
                sampleMode = false;
            }
            else{
                sampleMode = true;
            }

            if(sampleMode)
            {
                telemetry.addLine("SAMPLE MODE");
            }
            else
            {
                telemetry.addLine("SPECIMAN MODE");
            }

            //claw operation
            boolean currentBumperState  = gamepad2.right_bumper;
            //hold right bumper to close the claw
            if (currentBumperState) {
                claw.close();
                //telemetry.addLine("Close");
            }
            else {
                claw.open();
                //telemetry.addLine("Open");
            }


            //drive train
            //if(gamepad1.left_bumper)
                //driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            //else
            //    driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            //TODO add rotation to claw
            telemetry.update();
        }
    }
}

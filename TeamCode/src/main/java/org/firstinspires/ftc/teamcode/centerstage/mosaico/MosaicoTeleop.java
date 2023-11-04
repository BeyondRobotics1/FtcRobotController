package org.firstinspires.ftc.teamcode.centerstage.mosaico;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@TeleOp(group = "mosaico")
//@Disabled
public class MosaicoTeleop extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MosaicoClaw claw = new MosaicoClaw(hardwareMap, this);
        MosaicoSlide slide = new MosaicoSlide(hardwareMap, this);


        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            drive.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if(gamepad2.right_bumper ){
                claw.close();
            }
            else if (!gamepad2.right_bumper ){
                claw.open();
            }

            //hold left bumper to keep the claw at down position
            if(!gamepad2.left_bumper && !gamepad2.dpad_down ){
                claw.rotate_down();
            }
            else if(gamepad2.left_bumper){
                claw.rotate_up();
            }
            else if(gamepad2.dpad_down){
                claw.rotate_middle();
            }
            //release left bumper to keep the claw at the up position
           slide.setPower(-gamepad2.left_stick_y);
        }
    }
}
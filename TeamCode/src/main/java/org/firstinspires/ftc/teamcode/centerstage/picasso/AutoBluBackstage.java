package org.firstinspires.ftc.teamcode.centerstage.picasso;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.MaxVelocityTuner;

import java.util.Vector;

/*
 * This is a simple routine to test translational drive capabilities.
 */

@Autonomous(group = "drive")
@Disabled
public class AutoBluBackstage extends LinearOpMode {
    public static double DISTANCE1 = 30.5; // in
    //Servo pixelplacer = hardwareMap.get(Servo.class, "pixelplacer"); //check this one

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
                SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .forward(44.6)
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .forward(5)
                .build();
        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineToLinearHeading(new Pose2d(44.6,32, Math.toRadians(-90)))
                .build();
        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .strafeRight(20)
                .build();
        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .back(4)
                .build();
        Trajectory tra = drive.trajectoryBuilder(trajectory5.end())
                .strafeRight(20)
                .build();


        waitForStart();
        drive.followTrajectory(trajectory1);
        sleep(100);
        //insert pixel placement
        sleep(100);
        drive.followTrajectory(trajectory2);
        sleep(100);
        drive.followTrajectory(trajectory3);
        sleep(100);
        drive.followTrajectory(trajectory4);
        sleep(100);
        drive.followTrajectory(trajectory5);
        sleep(500);
        drive.followTrajectory(tra);
        if (isStopRequested()) return;
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
        sleep(4000);
        while (!isStopRequested() && opModeIsActive()) ;
    }
}


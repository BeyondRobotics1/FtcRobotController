package org.firstinspires.ftc.teamcode.centerstage.mosaico;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.MaxVelocityTuner;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class MosaicoAuto1 extends LinearOpMode {
    public static double DISTANCE1 = 30.5; // in
    public static double DISTANCE2 = 5; // in

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        MosaicoClaw claw = new MosaicoClaw(hardwareMap, this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .back(DISTANCE1)
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .forward(DISTANCE2)
                .build();


        waitForStart();

        if (isStopRequested()) return;
        claw.close();
        sleep(100);
        drive.followTrajectory(trajectory1);
        sleep(200);
        claw.open();
        sleep(200);
        drive.followTrajectory(trajectory2);
        sleep(100);
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
        sleep(4000);
        while (!isStopRequested() && opModeIsActive()) ;
    }
}


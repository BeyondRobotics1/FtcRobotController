package org.firstinspires.ftc.teamcode.intothedeep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@Autonomous(name = "RoadrunnerSpecimen (IntoTheDeep)", group = "Linear Opmode")
//@Disabled
public class RoadrunnerSpecimen extends LinearOpMode {

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
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initializing drive train");
        telemetry.update();
        Pose2d initialPosition = new Pose2d(-64.5, -7.75, 180);
        PinpointDrive driveTrain = new PinpointDrive(hardwareMap, initialPosition);

        ElapsedTime timer = new ElapsedTime();

        telemetry.addLine("Initializing slide arm");
        telemetry.update();
        Arm arm = new Arm(hardwareMap, this);
        arm.runWithEncoder();

        telemetry.addLine("Initializing slide");
        telemetry.update();
        Slide slide = new Slide(hardwareMap, this);
        slide.runWithEncoder();
        Slide.SlideTargetPosition slideOp = Slide.SlideTargetPosition.HIGH_BASkET;


        telemetry.addLine("Initializing intake arm");
        telemetry.update();
        Intake intake = new Intake(hardwareMap, this);


        telemetry.addLine("Initializing claw");
        telemetry.update();
        Claw claw = new Claw(hardwareMap, this);
        claw.close();
        //Goes to score first specimen
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading(new Pose2d(-36.5, -3, 180), Math.toRadians(180))
                        .waitSeconds(0.25).build()
        );
        sleep(200);
        //Pushes the  first, second, and third sample to the human player to clip on
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading(new Pose2d(-50, -3, 180), Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-18, -48, 180), Math.toRadians(180))
                        .lineToX(-57)
                        .splineToLinearHeading(new Pose2d(-18, -58, 180), Math.toRadians(180))
                        .lineToX(-57)
                        .splineToLinearHeading(new Pose2d(-18, -63, 180), Math.toRadians(180))
                        .lineToX(-57)
                        .waitSeconds(0.25).build());
        sleep(200);
        //Goes to grab the 1st specimen
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading( new Pose2d(-60, -33, 180), Math.toRadians(180))
                        .waitSeconds(0.25).build());
        sleep(200);
        //Goes to score the 1st  specimen
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading( new Pose2d(-36.5, -3, 180), Math.toRadians(180))
                        .waitSeconds(0.25).build());
        sleep(200);
        //Goes to grab the 2nd specimen
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading( new Pose2d(-60, -33, 180), Math.toRadians(180))
                        .waitSeconds(0.25).build());
        sleep(200);
        //Goes to score the 2nd  specimen
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading( new Pose2d(-36.5, -3, 180), Math.toRadians(180))
                        .waitSeconds(0.25).build());
        sleep(200);
        //Goes to grab the 3rd specimen
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading( new Pose2d(-60, -33, 180), Math.toRadians(180))
                        .waitSeconds(0.25).build());
        sleep(200);
        //Goes to score the 3rd  specimen
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading( new Pose2d(-36.5, -3, 180), Math.toRadians(180))
                        .waitSeconds(0.25).build());
        sleep(200);
        if(isStopRequested()){
            return;
        }






    }
}
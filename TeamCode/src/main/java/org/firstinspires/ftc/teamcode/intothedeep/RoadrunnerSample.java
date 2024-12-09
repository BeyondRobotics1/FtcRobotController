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

@Autonomous(name = "RoadrunnerSample (IntoTheDeep)", group = "Linear Opmode")
//@Disabled
public class RoadrunnerSample extends LinearOpMode {

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
        Pose2d initialPosition = new Pose2d(-62.5, 41.5, 0);
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
        //Goes to basket score first sample
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading(new Pose2d(-56,  55,-46), Math.toRadians(-46))
                        .waitSeconds(0.25).build()
        );

        sleep(200);
        //Goes to get second Sample
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading(new Pose2d(-50.5, 49, 0), Math.toRadians(0)) //-55, 48
                        .waitSeconds(0.25).build()
        );
        sleep(200);
        //Goes to score second Sample
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading(new Pose2d(-56,  55,-46), Math.toRadians(-46))
                        //-55, 48
                        .waitSeconds(0.25).build());
        sleep(200);
        //Goes to get third Sample
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading(new Pose2d(-50.5, 58, 0), Math.toRadians(0))
                        //-55, 48
                        .waitSeconds(0.25).build());
        sleep(200);
        //Goes to score third sample
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading(new Pose2d(-56,  55,-46), Math.toRadians(-46))
                        //-55, 48
                        .waitSeconds(0.25).build());
        sleep(200);
        //Goes to get fourth sample
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading(new Pose2d(-48.5,  57, 27.5), Math.toRadians(27.5))
                        //-55, 48
                        .waitSeconds(0.25).build());
        sleep(200);
        //Goes to score fourth  sample
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading(new Pose2d(-56,  55,-46), Math.toRadians(-46))
                        //-55, 48
                        .waitSeconds(0.25).build());
        sleep(200);
        //Goes to substation
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading(new Pose2d(-8,  24,-90), Math.toRadians(-90))
                        //-55, 48
                        .waitSeconds(0.25).build());
        sleep(200);
        if (isStopRequested()) return;







    }
}






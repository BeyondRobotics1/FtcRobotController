package org.firstinspires.ftc.teamcode.intothedeep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

import java.io.SequenceInputStream;
import java.util.Arrays;

@Autonomous(name = "RoadrunnerSpecimen", group = "Linear Opmode")
@Disabled

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
        VelConstraint twentyVel = (robotPose, _path, _disp) -> {
            if (robotPose.position.x.value() > -36.5) {
                return 20.0;
            } else {
                return 50.0;
            }
        };
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d startPose = new Pose2d(-64.5, -7.75, Math.toRadians(0));
        Pose2d scorePose = new Pose2d(-32.5, 0, Math.toRadians(0));
        Pose2d curvePoint = new Pose2d(-44, -3, Math.toRadians(180));
        Pose2d sampleOne = new Pose2d(-42.5, -31, Math.toRadians(-55));
        Pose2d sampleTwo = new Pose2d(-42.5, -43, Math.toRadians(-50));
        Pose2d sampleThree = new Pose2d(-44.5, -51, Math.toRadians(-50));
        Pose2d grab = new Pose2d(-60, -33, Math.toRadians(180));
        telemetry.addLine("Initializing drive train");
        telemetry.update();
        PinpointDrive driveTrain = new PinpointDrive(hardwareMap, startPose);


        ElapsedTime timer = new ElapsedTime();

        telemetry.addLine("Initializing slide arm");
        telemetry.update();


        OuttakeArm outtake = new OuttakeArm(hardwareMap, this);
        telemetry.addLine("Initializing slide");
        telemetry.update();
        Slide slide = new Slide(hardwareMap, this);
        slide.runWithEncoder();
        Slide.SlideTargetPosition slideOp = Slide.SlideTargetPosition.SPECIMEN_DELIVERY;

        telemetry.addLine("Initializing intake arm");
        telemetry.update();
        Intake intake = new Intake(hardwareMap, this);

        telemetry.addLine("Initializing claw");
        telemetry.update();
        Claw claw = new Claw(hardwareMap, this);
        claw.close();
        TrajectoryActionBuilder score = driveTrain.actionBuilder(driveTrain.pose) //use trajectoryactionbuilder to make your trajectories
                .splineToLinearHeading(scorePose, Math.toRadians(180), twentyVel);
        Action scoreFirst = score.build();
        waitForStart();
        //restart the timer
        timer.reset();

        //sleep(100);

        ///////////////////////////
        //preloaded

        //goes to score first specimen
//        //slide, arm, claw action here

        Actions.runBlocking(new SequentialAction(
                slide.autoToSpecimen(),
                new SleepAction(0.5),
                new ParallelAction(
                outtake.autoToSpecimenScore(),
                scoreFirst,
                slide.autoToSpecimenOne()
                ),
                new SleepAction(2),
                slide.slideDown(),
                new SleepAction(0.5),
                claw.openClaw()
                )
        );



//        intake.MoveToStartPosition();
//        arm.rotateToTargetAngleWithoutWaiting(Arm.ArmTargetAngle.OUTTAKE, -1);
//        sleep(600);
//
//        slide.moveToWithoutWaiting(6, 1);
//
//        sleep(1000);

//        Actions.runBlocking(
//                driveTrain.actionBuilder(initialPosition) //use trajectoryactionbuilder to make your trajectories
//                        .lineToX(38) //this is used for heading setting and reaching x position
//                        .waitSeconds(0.25).build()
//        );
//
//
//        sleep(300);
//        claw.open();
//        sleep(400);

//        arm.rotateToTargetAngleWithoutWaiting(Arm.ArmTargetAngle.INTAKE, -0.5);
//        sleep(400);
//
//
//        Actions.runBlocking(
//                driveTrain.actionBuilder(driveTrain.pose)
//                        .splineToLinearHeading(new Pose2d(60, 48, 0), Math.toRadians(0)) //-55, 48
//                        .waitSeconds(0.25).build()
//        );



        sleep(2000);


        if (isStopRequested()) return;


    }
}

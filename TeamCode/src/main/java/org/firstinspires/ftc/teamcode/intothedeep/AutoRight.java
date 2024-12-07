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

@Autonomous(name = "Auto Right (IntoTheDeep)", group = "Linear Opmode")
//@Disabled

public class AutoRight extends LinearOpMode {

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
        Pose2d initialPosition = new Pose2d(-64,-8,0);
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


        waitForStart();
        //restart the timer
        timer.reset();

        //sleep(100);

        ///////////////////////////
        //preloaded

        //main scoring function, can be reused to score (run this after you pick up a sample)
        Actions.runBlocking(
                driveTrain.actionBuilder(initialPosition) //use trajectoryactionbuilder to make your trajectories
                        //.strafeTo(new Vector2d(36, 4)) //this is used for heading setting and reaching x position
                        .splineToLinearHeading(new Pose2d(-58, -48, 0), Math.toRadians(0))
                        //.waitSeconds(0.25)
                        //.turn(45)
                        .waitSeconds(0.25).build()
        );

//        //slide, arm, claw action here
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

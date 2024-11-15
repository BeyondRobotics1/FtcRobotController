package org.firstinspires.ftc.teamcode.intothedeep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PathBuilder;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "AutoLeft (IntoTheDeep)", group = "Linear Opmode")
//@Disabled
public class AutoLeft extends LinearOpMode {

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
        Pose2d initialPosition = new Pose2d(-64,39.5,0);
        PinpointDrive driveTrain = new PinpointDrive(hardwareMap, initialPosition);

        ElapsedTime timer = new ElapsedTime();

        telemetry.addLine("Initializing slide");
        telemetry.update();
        Slide slide = new Slide(hardwareMap, this);
        slide.runWithEncoder();
        Slide.SlideTargetPosition slideOp = Slide.SlideTargetPosition.HIGH_BASkET;

        telemetry.addLine("Initializing slide arm");
        telemetry.update();
        Arm arm = new Arm(hardwareMap, this);
        arm.runWithEncoder();

        telemetry.addLine("Initializing intake arm");
        telemetry.update();
        Intake intake = new Intake(hardwareMap, this);


        telemetry.addLine("Initializing claw");
        telemetry.update();
        Claw claw = new Claw(hardwareMap, this);
        claw.close();
        TrajectoryActionBuilder score = driveTrain.actionBuilder(initialPosition) //use trajectoryactionbuilder to make your trajectories
                .lineToXSplineHeading(-54.375, Math.toRadians(-45)) //this is used for heading setting and reaching x position
                .waitSeconds(0.25)
                .strafeTo(new Vector2d(-54.375,61)) //-55.375,62 //this is used for moving to the correct y position (does not change heading)
                .waitSeconds(0.25);
        Action scoringTime = score.build();
        waitForStart();
        //restart the timer
        timer.reset();

        sleep(100);

        //main scoring function, can be reused to score (run this after you pick up a sample)
        Actions.runBlocking(
                new SequentialAction(
                        scoringTime
                )
        );
        //slide, arm, claw action here
        intake.MoveToAimingPosition();
        arm.rotateToTargetAngleWithoutWaiting(Arm.ArmTargetAngle.OUTTAKE, -1);
        sleep(300);

        slide.moveToPredefinedPositionWithoutWaiting(slideOp, 1);

        sleep(1200);
        intake.MoveToOuttakePosition();
        sleep(100);
        claw.open();
        sleep(10000);

        //score function ends here



        if (isStopRequested()) return;


    }
}

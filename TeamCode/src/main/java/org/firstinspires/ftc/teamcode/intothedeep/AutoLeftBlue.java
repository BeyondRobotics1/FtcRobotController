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

@Autonomous(name = "AutoLeftBlue (IntoTheDeep)", group = "Linear Opmode")
//@Disabled
public class AutoLeftBlue extends LinearOpMode {

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

        ElapsedTime timer = new ElapsedTime();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initializing drive train");
        telemetry.update();
        Pose2d initialPosition = new Pose2d(-63.3,39.2,0);
        PinpointDrive driveTrain = new PinpointDrive(hardwareMap, initialPosition);

        telemetry.addLine("Initializing slide");
        telemetry.update();
        Slide slide = new Slide(hardwareMap, this);
        slide.runWithEncoder();



        telemetry.addLine("Initializing outtake arm");
        telemetry.update();
        OuttakeArm outtakeArm = new OuttakeArm(hardwareMap, this);

        telemetry.addLine("Initializing intake");
        telemetry.update();
        Intake intake = new Intake(hardwareMap, this);
        intake.MoveToOuttakePosition();

        telemetry.addLine("Initializing intake slide");
        telemetry.update();
        IntakeSlide intakeSlide = new IntakeSlide(hardwareMap);
        intakeSlide.Move(0.498);

        telemetry.addLine("Initializing claw");
        telemetry.update();
        Claw claw = new Claw(hardwareMap, this);
        claw.close();

        waitForStart();
        //restart the timer
        timer.reset();

        //Goes to basket score preloaded sample
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                .strafeTo(new Vector2d(-50,57.5)) //-51.3,58.7 //this is used for moving to the correct y position (does not change heading)
                .waitSeconds(0.1).build()
        );
        PutSampleIntoBasket(driveTrain, slide, outtakeArm, claw);

        //Goes to get first Sample
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading(new Pose2d(-49.25, 46.2, 0), Math.toRadians(0)) //-55, 48
                        .waitSeconds(0.1).build()
        );
        GetSampleBack(intake, intakeSlide);

        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .strafeTo(new Vector2d(-50,57.5)) //-51.3,58.7 //this is used for moving to the correct y position (does not change heading)
                        .waitSeconds(0.1).build()
        );
        Score(driveTrain, intake, slide, outtakeArm, claw);;

        //second sample score
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading(new Pose2d(-48.25, 53.3, Math.toRadians(4.25)), Math.toRadians(0))
                        .waitSeconds(0.1).build()//0.25
        );
        GetSampleBack(intake, intakeSlide);

        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .strafeTo(new Vector2d(-49,51)) //-50, 57.5 score
                        //.splineToLinearHeading(new Pose2d(-50,57.5, 0), Math.toRadians(0))
                        .waitSeconds(0.1).build()
        );
        Score(driveTrain, intake, slide, outtakeArm, claw);;

        //get third sample
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading(new Pose2d(-51.5, 44.5, Math.toRadians(44)), Math.toRadians(0))
                        .waitSeconds(0.1).build()//0.25
        );
        GetSampleBack(intake, intakeSlide);

        intake.SetIntakeSpinner(Intake.IntakeMode.IDLE);

        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading(new Pose2d(-50,56.5, 0), Math.toRadians(0))
                        .waitSeconds(0.1).build()
        );
        Score(driveTrain, intake, slide, outtakeArm, claw);

        //parking
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .strafeTo(new Vector2d(-4, 34))
                        //.turn(Math.toRadians(-45))
                        .strafeTo(new Vector2d(-4, 12)).build()
        );
        intake.SetIntakeSpinner(Intake.IntakeMode.IDLE);

        sleep(10000);


        if (isStopRequested()) return;


    }

    private void GetSampleBack(Intake intake,
                               IntakeSlide intakeSlide

    )
    {
        intake.MoveToIntakePosition();
        intake.SetIntakeSpinner(Intake.IntakeMode.IN);
        intakeSlide.Move(0.6);
        sleep(1400);//1400

        intake.MoveToOuttakePositionAuto();
        intakeSlide.Move(0.35);
    }

    private void Score(PinpointDrive driveTrain,
                                     Intake intake,
                                     Slide slide,
                                     OuttakeArm outtakeArm,
                                     Claw claw

    )
    {
        intake.SetIntakeSpinner(Intake.IntakeMode.IN);
        sleep(300);//
        claw.close();
        sleep(200);

        PutSampleIntoBasket(driveTrain, slide, outtakeArm, claw);
    }

    private void PutSampleIntoBasket(PinpointDrive driveTrain,
                                     Slide slide,
                                     OuttakeArm outtakeArm,
                                     Claw claw

    )
    {
        slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.SPECIMEN_DELIVERY, 1);
        sleep(200);
        slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.HIGH_BASkET, 1);
        sleep(200);
        outtakeArm.Rotate(outtakeArm.SAMPLE_DELIVERY_POSITION);
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .turn(Math.toRadians(-45)) //
                        .waitSeconds(0.1).build()
        );
        sleep(200);
        claw.open();
        sleep(300);//400
        outtakeArm.Rotate(outtakeArm.SAMPLE_PICKUP_POSITION);
        sleep(700);//800
        slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.DOWN, 1);
    }

}

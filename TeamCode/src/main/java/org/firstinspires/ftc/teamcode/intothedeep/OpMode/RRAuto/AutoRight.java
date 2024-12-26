package org.firstinspires.ftc.teamcode.intothedeep.OpMode.RRAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.IntakeSlide;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.Slide;

@Autonomous(name = "Auto Right (IntoTheDeep)", group = "Linear Opmode")
public class AutoRight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        VelConstraint seventyVel = (robotPose, _path, _disp) -> {
            return 70;
        };

        ElapsedTime timer = new ElapsedTime();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initializing drive train");
        telemetry.update();
        Pose2d initialPosition = new Pose2d(-63.5, -7.75,0);
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


        telemetry.addLine("Initializing intake slide");
        telemetry.update();
        IntakeSlide intakeSlide = new IntakeSlide(hardwareMap);
        intakeSlide.Move(0.49);

        telemetry.addLine("Initializing claw");
        telemetry.update();
        Claw claw = new Claw(hardwareMap, this);
        claw.close();

        waitForStart();
        //restart the timer
        timer.reset();


// 1
        outtakeArm.Rotate(outtakeArm.SPECIMEN_READY_POSITION);
        slide.moveToWithoutWaiting(3., 1);
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .strafeTo(new Vector2d(-32.75,-2)) //-32,-2 //this is used for moving to the correct y position (does not change heading)
                        .waitSeconds(0.1).build()
        );
        outtakeArm.Rotate(outtakeArm.SPECIMEN_SCORE_POSITION);
        sleep(300);
        slide.moveToWithoutWaiting(0, 1);
        sleep(1000);
        claw.open();
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .strafeTo(new Vector2d(-49,-49), seventyVel) //-32,-2 //this is used for moving to the correct y position (does not change heading)
                        .waitSeconds(0.1).build()
        );

        intake.MoveToIntakePosition();
        sleep(200);
        outtakeArm.Rotate(outtakeArm.SAMPLE_PICKUP_POSITION);
        intake.SetIntakeSpinner(Intake.IntakeMode.IN);
        intakeSlide.Move(0.6);
        sleep(1400);//1400

        intake.MoveToOuttakePositionAuto();
        intakeSlide.Move(0.35);
        intake.SetIntakeSpinner(Intake.IntakeMode.IDLE);
        sleep(800);
        claw.close();
        sleep(100);
        outtakeArm.Rotate(outtakeArm.SPECIMEN_PICKUP_POSITION);
        intake.MoveToIntakePosition();
        sleep(800);
        claw.open();
        sleep(600);
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .strafeTo(new Vector2d(-49,-60.5), seventyVel)
                        .waitSeconds(0.1).build()
        );
        sleep(600);
        intake.SetIntakeSpinner(Intake.IntakeMode.IN);
        outtakeArm.Rotate(outtakeArm.SAMPLE_PICKUP_POSITION);
        intakeSlide.Move(0.6);
        sleep(1400);
        intake.MoveToOuttakePositionAuto();
        intakeSlide.Move(0.35);
        intake.SetIntakeSpinner(Intake.IntakeMode.IDLE);
        sleep(800);
        claw.close();
        sleep(100);
        intake.MoveToIntakePosition();
        outtakeArm.Rotate(outtakeArm.SPECIMEN_PICKUP_POSITION);
        sleep(800);
        claw.open();

        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading(new Pose2d(-32,-52.5,Math.toRadians(-45)),0, seventyVel)
                        .waitSeconds(0.1).build()
        );
        sleep(400);
        intake.SetIntakeSpinner(Intake.IntakeMode.IN);
        outtakeArm.Rotate(outtakeArm.SAMPLE_PICKUP_POSITION);
        intakeSlide.Move(0.6);
        sleep(1400);
        intake.MoveToOuttakePositionAuto();
        intakeSlide.Move(0.35);
        intake.SetIntakeSpinner(Intake.IntakeMode.IDLE);
        sleep(800);
        claw.close();
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineToLinearHeading(new Pose2d(-49,-60.5,0),0, seventyVel)
                        .waitSeconds(0.1).build()
        );
        sleep(600);
        intake.MoveToIntakePosition();
        outtakeArm.Rotate(outtakeArm.SPECIMEN_PICKUP_POSITION);
        sleep(800);
        claw.open();
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .strafeTo(new Vector2d(-45,-50.5),seventyVel)
                        .waitSeconds(0.1).build()
        );
        sleep(3000);
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .strafeTo(new Vector2d(-55,-40.5))
                        .waitSeconds(0.1).build()
        );

//        sleep(200);
//        claw.close();
//        Actions.runBlocking(
//                driveTrain.actionBuilder(driveTrain.pose)
//                        .strafeTo(new Vector2d(-))
//        );




        sleep(10000);

        if (isStopRequested()) return;


    }
}

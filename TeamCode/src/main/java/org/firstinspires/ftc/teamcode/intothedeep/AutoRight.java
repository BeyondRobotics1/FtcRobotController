package org.firstinspires.ftc.teamcode.intothedeep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
        intakeSlide.Move(0.498);

        telemetry.addLine("Initializing claw");
        telemetry.update();
        Claw claw = new Claw(hardwareMap, this);
        claw.close();

        waitForStart();
        //restart the timer
        timer.reset();

        outtakeArm.Rotate(outtakeArm.SPECIMEN_READY_POSITION);
        slide.moveToWithoutWaiting(3., 1);
        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .strafeTo(new Vector2d(-33,-2)) //-32,-2 //this is used for moving to the correct y position (does not change heading)
                        .waitSeconds(0.1).build()
        );
        outtakeArm.Rotate(outtakeArm.SPECIMEN_SCORE_POSITION);
        sleep(300);
        slide.moveToWithoutWaiting(0, 1);


        sleep(10000);

        if (isStopRequested()) return;


    }
}

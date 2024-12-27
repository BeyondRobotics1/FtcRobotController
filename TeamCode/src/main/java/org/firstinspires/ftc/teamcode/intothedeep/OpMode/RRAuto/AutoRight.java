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
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.ClawRotor;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.IntakeSlide;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.Slide;

@Autonomous(name = "Auto Right (IntoTheDeep)", group = "A Into the Deep")
@Disabled
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


        telemetry.addLine("Initializing claw rotor");
        telemetry.update();
        ClawRotor clawRotor = new ClawRotor(hardwareMap, this);
        clawRotor.SetClawDown();

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

        Actions.runBlocking(
                driveTrain.actionBuilder(driveTrain.pose)
                        .strafeTo(new Vector2d(-50,-20), seventyVel) //-51.3,58.7 //this is used for moving to the correct y position (does not change heading)
                        .waitSeconds(0.1).build()
        );


//        //Goes to get first Sample
//        Actions.runBlocking(
//                driveTrain.actionBuilder(driveTrain.pose)
//                        .splineToLinearHeading(new Pose2d(-50, -10, Math.toRadians(-45)),
//                                Math.toRadians(0)) //-38.25, -36
//                        .waitSeconds(0.01).build()
//        );

        sleep(10000);

        if (isStopRequested()) return;


    }
}

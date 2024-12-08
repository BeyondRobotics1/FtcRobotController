package org.firstinspires.ftc.teamcode.intothedeep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Pivot Servo Position Test", group = "Into the Deep")
public class PivotServoPositionTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initializing drive train");
        telemetry.update();
        PinpointDrive driveTrain = new PinpointDrive(hardwareMap, new Pose2d(0,0, 0));

        telemetry.addLine("Initializing slide");
        telemetry.update();
        Slide slide = new Slide(hardwareMap, this);
        slide.runWithEncoder();

        telemetry.addLine("Initializing claw");
        telemetry.update();
        Claw claw = new Claw(hardwareMap, this);

        telemetry.addLine("Initializing outtake arm");
        telemetry.update();
        OuttakeArm outtakeArm = new OuttakeArm(hardwareMap, this);

        telemetry.addLine("Initializing intake");
        telemetry.update();
        Intake intake = new Intake(hardwareMap, this);

        telemetry.addLine("Initializing intake slide");
        telemetry.update();
        IntakeSlide intakeSlide = new IntakeSlide(hardwareMap);

        waitForStart();

        //slide is manually controlled
        Slide.SlideTargetPosition slideOp = Slide.SlideTargetPosition.MANUAL;

        boolean robotCentric = true;

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            //test the pivot servo, DON"T test together with fourbar servo
            double pivotPosition = gamepad1.left_trigger*0.7;
            telemetry.addData("Pivot Servo position", Math.abs(pivotPosition));
            intake.TestPivotServo(pivotPosition);

            //TODO add rotation to claw
            telemetry.update();
        }
    }
}

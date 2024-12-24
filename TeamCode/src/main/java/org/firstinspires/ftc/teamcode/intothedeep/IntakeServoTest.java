package org.firstinspires.ftc.teamcode.intothedeep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Intake Servo Test", group = "Into the Deep")
public class IntakeServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initializing intake");
        telemetry.update();
        Intake intake = new Intake(hardwareMap, this);

        telemetry.addData("gamepad1.x", "Test 4-Bar Servo." );
        telemetry.addData("gamepad1.y", "Test Pivot Servo." );


        telemetry.addData("gamepad1.left_trigger", "Change Pivot or 4-Bar Servo position." );
        telemetry.addData("gamepad1.left_bumper", "Spin Servo intake." );
        telemetry.addData("gamepad1.right_bumper", "Spin Servo spit out." );

        telemetry.update();

        Servo servo = null;

        waitForStart();

        //slide is manually controlled
        Slide.SlideTargetPosition slideOp = Slide.SlideTargetPosition.MANUAL;

        boolean robotCentric = true;

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            if(servo == null) {
                if (gamepad1.x) {
                    servo = hardwareMap.get(Servo.class, "intakeFourBar");
                    servo.setDirection(Servo.Direction.REVERSE);
                }
                else if (gamepad1.y)
                    servo = hardwareMap.get(Servo.class, "intakePivot");
            }

            if(servo != null) {
                //use gamepad1 left trigger to set servo positions dynamically
                double pivotPosition = gamepad1.left_trigger;
                telemetry.addData("Servo position", Math.abs(pivotPosition));
                servo.setPosition(pivotPosition);
            }

            //intake control
            //left bumper spit out
            if(gamepad1.left_bumper)
                intake.SetIntakeSpinner(Intake.IntakeMode.IN);
            else if (gamepad1.right_bumper) //right bumper take in
                intake.SetIntakeSpinner(Intake.IntakeMode.OUT);
            else //otherwise, idle to save energy
                intake.SetIntakeSpinner(Intake.IntakeMode.IDLE);

            //TODO add rotation to claw
            telemetry.update();
        }
    }
}

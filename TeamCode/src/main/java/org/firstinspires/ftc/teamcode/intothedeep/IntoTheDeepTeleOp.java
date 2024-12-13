package org.firstinspires.ftc.teamcode.intothedeep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.common.Helper;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Into the Deep", group = "Into the Deep")

public class IntoTheDeepTeleOp extends LinearOpMode {

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

            //double newValue = (value - oldMin) * (newMax - newMin) / (oldMax - oldMin) + newMin;
            double slideOutSpeed = gamepad1.left_trigger;
            double slideInSpeed = gamepad1.right_trigger;
            if(slideOutSpeed >= 0.1) {

                telemetry.addData("left_trigger", slideOutSpeed);

                //scale from [0 1] to [0.5 1], move out, (slideOutSpeed + 1) * 0.5

                //now since squared, the number could be less than 0.5, which will
                //pull the slide back
                slideOutSpeed = Helper.squareWithSign((slideOutSpeed + 1) * 0.5);

                //cap the retraction and push power into the desired range
                if(slideOutSpeed < 0.43)
                    slideOutSpeed = 0.43;

                if(slideOutSpeed > 0.7)
                    slideOutSpeed = 0.7;

                intakeSlide.Move(slideOutSpeed);

                telemetry.addData("slideOutSpeed", slideOutSpeed);
            }
            else if(slideInSpeed > 0.1) {

                //when retracting back, move the intake to the outtake position
                //for the claw to pick the sample up
                intake.MoveToOuttakePosition();

                //scale from [0 1] to [0 0.5], move in
                intakeSlide.Move(0.5-slideInSpeed*0.5);

                telemetry.addData("slideInSpeed", 0.5-slideInSpeed*0.5);
            }
            else
                intakeSlide.Move(0.485);

            //intake control
            //left bumper spit out
            if(gamepad1.left_bumper)
                intake.SetIntakeSpinner(Intake.IntakeMode.IN);
            else if (gamepad1.right_bumper) //right bumper take in
                intake.SetIntakeSpinner(Intake.IntakeMode.OUT);
            else //otherwise, idle to save energy
                intake.SetIntakeSpinner(Intake.IntakeMode.IDLE);

            //intake sample container
            if(gamepad1.a)
                intake.MoveToOuttakePosition();
            else if (gamepad1.y)
                intake.MoveToIntakePosition();
            else if (gamepad1.x)
                intake.MoveToHeadDownPosition();

            //slide operation
            //By holding the left bumper, manual operation
            if (gamepad2.left_bumper) {
                slide.setPower(-gamepad2.left_stick_y);
                slideOp = Slide.SlideTargetPosition.MANUAL;
            } else { //by default, using dpad to move the slide to the predefined positions

                if (gamepad2.dpad_down && slideOp != Slide.SlideTargetPosition.DOWN) {
                    slideOp = Slide.SlideTargetPosition.DOWN;
                    slide.moveToPredefinedPositionWithoutWaiting(slideOp = Slide.SlideTargetPosition.DOWN, 1); //ground
                } else if (gamepad2.dpad_left && slideOp != Slide.SlideTargetPosition.SPECIMEN_DELIVERY) {
                    slideOp = Slide.SlideTargetPosition.SPECIMEN_DELIVERY;
                    slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.SPECIMEN_DELIVERY, 1); //low
                } else if (gamepad2.dpad_up && slideOp != Slide.SlideTargetPosition.HIGH_BASkET) {
                    slideOp = Slide.SlideTargetPosition.HIGH_BASkET;
                    slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.HIGH_BASkET, 1);
                }
            }

            //outtake arm operation
            //use left trigger for manual claw arm control
            if (Math.abs(gamepad2.left_trigger) > 0.5) {

                //pull
                double position = gamepad2.right_stick_y;

                if (position > 0)
                    position = 0;
                outtakeArm.Rotate(1 - Math.abs(position));
                //telemetry.addData("Servo position", 1 - Math.abs(position));
            }
            else //by default, use a, x, y, b buttons to control the claw arm
            {
                //button a, set the arm to pickup a sample from the intake
                if(gamepad2.a)
                    outtakeArm.Rotate(outtakeArm.SAMPLE_PICKUP_POSITION);
                else if(gamepad2.x) //button x, set the arm to pickup specimen from human player
                    outtakeArm.Rotate(outtakeArm.SPECIMEN_PICKUP_POSITION);
                else if(gamepad2.y) //button y, set the arm to the specimen score position
                    outtakeArm.Rotate(outtakeArm.SPECIMEN_SCORE_POSITION);
                else if(gamepad2.b) //button b, set the arm to score samples into high basket
                    outtakeArm.Rotate(outtakeArm.SAMPLE_DELIVERY_POSITION);
            }


            //claw operation
            boolean currentBumperState  = gamepad2.right_bumper;
            //hold right bumper to close the claw
            if (currentBumperState) {
                claw.close();
            }
            else {
                claw.open();
            }


            //drive train
            //if(gamepad1.left_bumper)
                driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            //else
            //    driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            //TODO add rotation to claw
            telemetry.update();
        }
    }
}

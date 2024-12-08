package org.firstinspires.ftc.teamcode.intothedeep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.powerplay.TeleOp1;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Into the deep", group = "TeleOp")

public class TeleOp extends LinearOpMode {

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


        //arm is manually controlled

        boolean robotCentric = true;

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            //intake slide control
            if(gamepad1.dpad_up){
                intakeSlide.MoveOut();
            } else if (gamepad1.dpad_down){
                intakeSlide.MoveIn();
            } else {
                intakeSlide.Hold();
            }

            //intake control
            //right bumper spit out
            if(gamepad1.right_bumper)
                intake.SetIntakeSpinner(Intake.IntakeMode.OUT);
            else if (gamepad1.left_bumper) //left bumper take in
                intake.SetIntakeSpinner(Intake.IntakeMode.IN);
            else //otherwise, idle to save energy
                intake.SetIntakeSpinner(Intake.IntakeMode.IDLE);

            //intake sample container
            if(gamepad1.a)
                intake.MoveToOuttakePosition();
            else if (gamepad1.y)
                intake.MoveToIntakePosition();
            else if (gamepad1.x)
                intake.MoveToHeadDownPosition();

//            //test the fourbar servo, DON"T test together with pivot servo
//            double fourBarPosition = gamepad1.left_trigger;
//            telemetry.addData("FourBar Servo position", Math.abs(fourBarPosition));
//            intake.TestFourBarServo(fourBarPosition);

//            //test the pivot servo, DON"T test together with fourbar servo
//            double pivotPosition = gamepad1.right_trigger*0.5;
//            telemetry.addData("Pivot Servo position", Math.abs(pivotPosition));
//            intake.TestPivotServo(pivotPosition);

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
                else if(gamepad2.b) //button b, set the arm to score samples into baskets
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

package org.firstinspires.ftc.teamcode.intothedeep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Into the deep", group = "TeleOp")

public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initializing drive train");
        telemetry.update();
        PinpointDrive driveTrain = new PinpointDrive(hardwareMap, new Pose2d(0,0, 0));

        telemetry.addLine("Initializing slide");
        telemetry.update();
        Slide slide = new Slide(hardwareMap, this);
        slide.runWithEncoder();

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


        waitForStart();

        //slide is manually controlled
        Slide.SlideTargetPosition slideOp = Slide.SlideTargetPosition.MANUAL;
        Arm.ArmTargetAngle armOp = Arm.ArmTargetAngle.MANUAL;

        //arm is manually controlled

        boolean robotCentric = true;

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {


            //telemetry.addData("Slide.SlideTargetPosition.DOWN", "%d index", Slide.SlideTargetPosition.DOWN.getValue());
            //telemetry.addData("Slide.SlideTargetPosition.LOW_BASKET", "%d index", Slide.SlideTargetPosition.LOW_BASKET.getValue());


            //slide operation
            if(gamepad2.left_bumper) {
                if (gamepad2.a) {
                    //when button a is clicked, go to the aiming position
                    // slide down, arm down, intake aiming

                    intake.MoveToAimingPosition();

                    if(slide.getSlideHeightInches() > 6) {
                        slideOp = Slide.SlideTargetPosition.DOWN;
                        slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.DOWN, 1.0);
                    }
                   sleep(200);

                    //arm down
                    if(armOp != Arm.ArmTargetAngle.INTAKE) {
                        armOp = Arm.ArmTargetAngle.INTAKE;
                        arm.rotateToTargetAngleWithoutWaiting(Arm.ArmTargetAngle.INTAKE, -1.0);
                    }

                    //slide out
                    if(slideOp != Slide.SlideTargetPosition.INTAKE){
                        slideOp = Slide.SlideTargetPosition.INTAKE;
                        slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.INTAKE, 1.0);
                    }


                } else if (gamepad2.x) {

                    //when x button is clicked, goto outtake position
                    //intake outtake position, slide down, arm up

                    intake.MoveToOuttakePosition(); //

                    if (slideOp != Slide.SlideTargetPosition.DOWN) {
                        slideOp = Slide.SlideTargetPosition.DOWN;
                        slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.DOWN, 1.0);
                    }

                    sleep(1000);

                    if (armOp != Arm.ArmTargetAngle.OUTTAKE) {
                        armOp = Arm.ArmTargetAngle.OUTTAKE;
                        arm.rotateToTargetAngleWithoutWaiting(Arm.ArmTargetAngle.OUTTAKE, -1.0);
                    }
                }
                else if (gamepad2.y) {
                    if(slideOp != Slide.SlideTargetPosition.HIGH_BASkET ){

                        //arm position is down
                        if(armOp != Arm.ArmTargetAngle.INTAKE)
                        {
                            slideOp = Slide.SlideTargetPosition.HIGH_BASkET;
                            slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.HIGH_BASkET, 1.0);
                        }
                    }
                }
                else if (gamepad2.b) { //intake position and close
                    if(slideOp == Slide.SlideTargetPosition.INTAKE){

                        intake.MoveToIntakePosition();//
                        sleep(100);
                        claw.close();

//                        if(armOp != Arm.ArmTargetAngle.INTAKE) {
//                            slideOp = Slide.SlideTargetPosition.LOW_BASKET;
//                            slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.LOW_BASKET, 1.0);
//                        }
                    }
                }

//                } else if (gamepad2.y && armOp != Arm.ArmTargetAngle.SPECIMEN) {
//                    armOp = Arm.ArmTargetAngle.SPECIMEN;
//                    arm.rotateToTargetAngleWithoutWaiting(Arm.ArmTargetAngle.SPECIMEN, -1.0);
//                } else if (gamepad2.b && armOp != Arm.ArmTargetAngle.HANG) {
//                    armOp = Arm.ArmTargetAngle.HANG;
//                    arm.rotateToTargetAngleWithoutWaiting(Arm.ArmTargetAngle.HANG, -1.0);
//                }
            }
            else
            {
                slideOp = Slide.SlideTargetPosition.MANUAL;
                slide.setPower(-gamepad2.left_stick_y);

                armOp = Arm.ArmTargetAngle.MANUAL;
                arm.setPower(gamepad2.right_stick_y);

                //intake operation
                if (gamepad2.dpad_down) {
                    intake.MoveToAimingPosition();
                }
                else if (gamepad2.dpad_left) {
                    intake.MoveToStartPosition();
                }
                else if (gamepad2.dpad_up){
                    intake.MoveToOuttakePosition();
                }
                else if (gamepad2.dpad_right) {
                    intake.MoveToIntakePosition();
                }
            }
            slide.autoMoveToWithoutWaitingLoop();
            arm.autoRotateToWithoutWaitingLoop();

            //claw operation
            boolean currentBumperState  = gamepad2.right_bumper;
            //hold right bumper to close the claw
            if (currentBumperState)
                claw.close();
            else
                claw.open();


            //drive train
            //if(gamepad1.left_bumper)
                driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            //else
            //    driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.update();
        }
    }
}

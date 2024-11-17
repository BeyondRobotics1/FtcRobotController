package org.firstinspires.ftc.teamcode.intothedeep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Into the deep", group = "TeleOp")

public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initializing drive train");
        telemetry.update();
        PinpointDrive driveTrain = new PinpointDrive(hardwareMap, new Pose2d(0,0, 0));

        telemetry.addLine("Initializing slide arm");
        telemetry.update();
        Arm arm = new Arm(hardwareMap, this);
        arm.runWithEncoder();

        telemetry.addLine("Initializing slide");
        telemetry.update();
        Slide slide = new Slide(hardwareMap, this, arm);
        slide.runWithEncoder();



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

        boolean sampleMode = true;

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            if(gamepad1.left_bumper)
            {
                sampleMode = false;
            }
            else{
                sampleMode = true;
            }

            if(sampleMode)
            {
                telemetry.addLine("SAMPLE MODE");
            }
            else
            {
                telemetry.addLine("SPECIMAN MODE");
            }

            telemetry.addData("Arm angle", "%f", arm.getArmAngle());
            telemetry.addData("Slide extension", "%f", slide.getSlideHeightInches());
            //telemetry.addData("Slide.SlideTargetPosition.LOW_BASKET", "%d index", Slide.SlideTargetPosition.LOW_BASKET.getValue());


            if(Math.abs(gamepad2.left_trigger) > 0.5) {
                //telemetry.addLine("claw turned");
                intake.ChangeClawDirection(true);
            }
            else {
                //telemetry.addLine("claw centered");
                intake.ChangeClawDirection(false);
            }


            //slide, arm, and intake manual operation
            if(gamepad2.left_bumper) {
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
            else
            {
                if (gamepad2.x) {
                    //when button a is clicked, go to the aiming position
                    // slide down, arm down, intake aiming

                    if(sampleMode)
                        intake.MoveToAimingPosition();
                    else
                        intake.MoveToStartPosition();

                    if(slideOp != Slide.SlideTargetPosition.INTAKE &&
                            slideOp != Slide.SlideTargetPosition.INTAKE1 &&
                            slideOp != Slide.SlideTargetPosition.INTAKE2) {
                        //if (slide.getSlideHeightInches() > 6) {
                        if(slideOp != Slide.SlideTargetPosition.DOWN)
                        {
                            slideOp = Slide.SlideTargetPosition.DOWN;
                            slide.moveToPredefinedPosition(Slide.SlideTargetPosition.DOWN, 1.0);

                            //sleep(800);
//                            //use a timer instead of sleeping entire bot
//                            timer.reset();
//                            if (timer.milliseconds() > 800){
//                                continue;
//                            }
                        }
                    }

                    //arm down
                    if(armOp != Arm.ArmTargetAngle.INTAKE) {
                        armOp = Arm.ArmTargetAngle.INTAKE;
                        //arm.rotateToTargetAngleWithoutWaiting(Arm.ArmTargetAngle.INTAKE, -0.5);
                        arm.rotateToTargetAngle(Arm.ArmTargetAngle.INTAKE, -0.5);

                        //sleep(400);

//                        //use a timer instead of sleeping the entire robot
//                        timer.reset();
//                        if (timer.milliseconds() > 400){
//                            continue;
//                        }
                    }

//                    //slide out
//                    if(slideOp != Slide.SlideTargetPosition.INTAKE){
//                        slideOp = Slide.SlideTargetPosition.INTAKE;
//                        slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.INTAKE, 1);//
//                    }


                } else if (gamepad2.b) {

                    //when x button is clicked, goto outtake position
                    //intake outtake position, slide down, arm up

                    if(sampleMode)
                        intake.MoveToOuttakePosition();
                    else
                        intake.MoveToStartPosition();


                    if (slideOp != Slide.SlideTargetPosition.DOWN) {
                        slideOp = Slide.SlideTargetPosition.DOWN;

                        slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.DOWN, 1);//
                        //slide.moveToPredefinedPosition(Slide.SlideTargetPosition.DOWN, 1);//

                        sleep(800);
//                        //use a timer instead of sleeping the rest of the robot
//                        timer.reset();
//                        if (timer.milliseconds() > 1000){
//                            continue;
//                        }
                    }

                    //arm up
                    if (armOp != Arm.ArmTargetAngle.OUTTAKE) {
                        armOp = Arm.ArmTargetAngle.OUTTAKE;
                        arm.rotateToTargetAngleWithoutWaiting(Arm.ArmTargetAngle.OUTTAKE, -1);//
                    }
                }
                else if (gamepad2.y) {
                    if(sampleMode) {
                        //slide up to high basket
                        if (slideOp != Slide.SlideTargetPosition.HIGH_BASkET) {

                            //arm position is down
                            if (armOp != Arm.ArmTargetAngle.INTAKE ) {
                                slideOp = Slide.SlideTargetPosition.HIGH_BASkET;
                                slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.HIGH_BASkET, 1.0);
                            }
                        }
                    }
                    else
                    {
                        //slide up to high basket
                        if (slideOp != Slide.SlideTargetPosition.SPECIMEN_DELIVERY) {

                            //arm position is down
                            if (armOp != Arm.ArmTargetAngle.INTAKE) {
                                slideOp = Slide.SlideTargetPosition.SPECIMEN_DELIVERY;
                                slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.SPECIMEN_DELIVERY, 1.0);
                            }
                        }
                    }
                }
                else if (gamepad2.a) {

                    //intake position, claw person should close the claw
                    //if failed to grab a sample, click button x back to aiming position
                    if(slideOp == Slide.SlideTargetPosition.DOWN||
                            slideOp == Slide.SlideTargetPosition.INTAKE ||
                            slideOp == Slide.SlideTargetPosition.INTAKE1 ||
                            slideOp == Slide.SlideTargetPosition.INTAKE2){

                        if(sampleMode)
                            intake.MoveToIntakePosition();
                        else
                            intake.MoveToStartPosition();

                    }
                }
                else if (gamepad2.dpad_down)
                {
                    if (slideOp != Slide.SlideTargetPosition.DOWN && Math.abs(arm.getArmAngle()) < 5) {
                        slideOp = Slide.SlideTargetPosition.DOWN;
                        slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.DOWN, 1);//
                    }
                }
                else if (gamepad2.dpad_left)
                {
                    if (slideOp != Slide.SlideTargetPosition.INTAKE1 && Math.abs(arm.getArmAngle()) < 5) {
                        slideOp = Slide.SlideTargetPosition.INTAKE1;
                        slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.INTAKE1, 1);//
                    }
                }
                else if (gamepad2.dpad_up)
                {
                    if (slideOp != Slide.SlideTargetPosition.INTAKE2 && Math.abs(arm.getArmAngle()) < 5) {
                        slideOp = Slide.SlideTargetPosition.INTAKE2;
                        slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.INTAKE2, 1);//
                    }
                }
                else if (gamepad2.dpad_right)
                {
                    if (slideOp != Slide.SlideTargetPosition.INTAKE && Math.abs(arm.getArmAngle()) < 5) {
                        slideOp = Slide.SlideTargetPosition.INTAKE;
                        slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.INTAKE, 1);//
                    }
                }


            }
            slide.autoMoveToWithoutWaitingLoop();
            arm.autoRotateToWithoutWaitingLoop();

            //claw operation
            boolean currentBumperState  = gamepad2.right_bumper;
            //hold right bumper to close the claw
            if (currentBumperState) {
                claw.close();
                //telemetry.addLine("Close");
            }
            else {
                claw.open();
                //telemetry.addLine("Open");
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

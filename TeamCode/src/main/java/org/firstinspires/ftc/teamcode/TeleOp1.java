package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.nio.channels.ConnectionPendingException;

@TeleOp
public class TeleOp1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //A digital touch sensor used to stop the slide moving too much
        DigitalChannel touchSensor =  hardwareMap.get(DigitalChannel.class, "slide_stopper");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);


        //drivetrain
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);

        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap, this);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //hold right bumper to close the claw
            if (gamepad2.right_bumper)
                claw.close();
            else //release right bumper to open the claw
                claw.open();

            //arm
            if (gamepad2.y) { //press Y, go to the front position
                arm.setTurretPosition(1);
            } else if (gamepad2.b) { //press b, go to the right position
                arm.setTurretPosition(2);
            } else if (gamepad2.x) { //press x, go to the left position
                arm.setTurretPosition(0);
            }

            telemetry.addData("touch sensor", touchSensor.getState());


            //use left stick y to set the power slide motors
            double slidePower = -gamepad2.left_stick_y;
            if(touchSensor.getState() == false){ //touch sensor is pushed
                if(slidePower < 0){ //slide is moving down, so then stop
                    arm.moveSlide(0);
                }else //slide is not moving down, don't stop
                    arm.moveSlide(slidePower);
            }else { //touch sensor is not pushed
                arm.moveSlide(slidePower);
            }

            double slide_height = arm.getDistanceINCH();
            telemetry.addData("Slide Height",slide_height );

            /*
            if (gamepad2.y) {
                arm.setTurretPosition(0.165); //front position
            } else if (gamepad2.x) {
                arm.setTurretPosition(0.265); //right position
            } else if (gamepad2.b) {
                arm.setTurretPosition(0.065); //left position
            }

            telemetry.addLine(String.format("\nDetected turret position=%f", arm.getTurretPosition()));
            */


            //drive train
            driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.update();

            sleep(100);
        }
    }
}

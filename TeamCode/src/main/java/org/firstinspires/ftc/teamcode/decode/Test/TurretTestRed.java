package org.firstinspires.ftc.teamcode.decode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.OpMode.DecodeBlackBoard;
import org.firstinspires.ftc.teamcode.decode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Turret;

@TeleOp(name = "Turret Tuner Red", group = "Decode Test")
public class TurretTestRed extends LinearOpMode {

    DriveTrain driveTrain;
    boolean isInitialPinpointPositionSet;
    boolean fieldCentric = true;

    @Override
    public void runOpMode() throws InterruptedException {

        //driveTrain = new DriveTrain(hardwareMap, this, false);

        isInitialPinpointPositionSet = false;
        Turret turret = new Turret(hardwareMap, this,
                DecodeBlackBoard.RED_RESET_POSE,//new Pose2D(DistanceUnit.INCH, 8, 111, AngleUnit.DEGREES, 0),
                DecodeBlackBoard.RED_TARGET_POSE,
                DecodeBlackBoard.RED,
                true,
                true, false);

//        double startingAngle = turret.getAnalogStartingAngle();
//        telemetry.addData("Analog start degree", startingAngle);
//        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            if(!isInitialPinpointPositionSet)
            {
                turret.setIMUPoseToRobotStartPose();
                isInitialPinpointPositionSet = true;
            }

//            if(gamepad1.left_bumper)
//            {
//                turret.calibrateTurret();
//            }
//            else
            if(gamepad1.right_bumper)
            {
                double pivotPosition = Math.abs(gamepad1.left_trigger);

                if(pivotPosition > 0.380){
                    pivotPosition = 0.380;
                }
                turret.setServoPosition(pivotPosition);
            }
            else if (gamepad1.a)
                turret.setServoPosition(0.195);
            else if (gamepad1.b)
                turret.setServoPosition(0.380);
            else if (gamepad1.x)
                turret.setServoPosition(0);
            else if (gamepad1.y)
                turret.autoAim(false);
            else
            {
                turret.autoAim(true);
            }

            if(gamepad1.dpadUpWasPressed())
                fieldCentric = !fieldCentric;

//            if(fieldCentric)
//                driveTrain.setPower2(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x,
//                        Math.toRadians(180+turret.getBotHeadingDegrees()));
//            else
//                driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


            //telemetry.addData("Turret Servo position", turret.getServoPosition());

            telemetry.update();
        }

    }





}

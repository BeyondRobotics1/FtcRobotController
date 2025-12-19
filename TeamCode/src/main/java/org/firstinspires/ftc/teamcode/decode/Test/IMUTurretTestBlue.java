package org.firstinspires.ftc.teamcode.decode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.decode.OpMode.DecodeBlackBoard;
import org.firstinspires.ftc.teamcode.decode.Subsystems.IMUTurret;

@TeleOp(name = "IMU Turret Tuner Blue", group = "Decode Test")
public class IMUTurretTestBlue extends LinearOpMode {

    boolean isInitialPinpointPositionSet;

    @Override
    public void runOpMode() throws InterruptedException {

        isInitialPinpointPositionSet = false;
        IMUTurret turret = new IMUTurret(hardwareMap, this,
                DecodeBlackBoard.BLUE_RESET_POSE,
                DecodeBlackBoard.BLUE_TARGET_POSE,
                DecodeBlackBoard.BLUE,
                true,
                true);

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

            if(gamepad1.left_bumper)
            {
                turret.calibrateTurret();
            }
            else if(gamepad1.right_bumper)
            {
                double pivotPosition = Math.abs(gamepad1.left_trigger);

                if(pivotPosition > 0.388){
                    pivotPosition = 0.388;
                }
                turret.setServoPosition(pivotPosition);
            }
            else if (gamepad1.a)
                turret.setServoPosition(0.194);
            else if (gamepad1.b)
                turret.setServoPosition(0.388);
            else if (gamepad1.x)
                turret.setServoPosition(0);
            else
            {
                turret.autoAim();
            }


            telemetry.addData("Turret Servo position", turret.getServoPosition());

            telemetry.update();
        }

    }





}

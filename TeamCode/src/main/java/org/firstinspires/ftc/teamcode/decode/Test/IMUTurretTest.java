package org.firstinspires.ftc.teamcode.decode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.decode.OpMode.DecodeBlackBoard;
import org.firstinspires.ftc.teamcode.decode.Subsystems.IMUTurret;

@TeleOp(name = "IMU Turret Test", group = "Decode Test")
public class IMUTurretTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        IMUTurret turret = new IMUTurret(hardwareMap, this,
                new Pose2D(DistanceUnit.INCH, 8, 48 + 7.5, AngleUnit.DEGREES, 0),
                DecodeBlackBoard.BLUE_TARGET_POSE,
                false);

//        double startingAngle = turret.getAnalogStartingAngle();
//        telemetry.addData("Analog start degree", startingAngle);
//        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            if(gamepad1.left_bumper)
                turret.autoAim();
            else if (gamepad1.a)
                turret.setServoPosition(0.21);
            else if (gamepad1.b)
                turret.setServoPosition(0.42);
            else if (gamepad1.x)
                turret.setServoPosition(0);
            else
            {
                double pivotPosition = Math.abs(gamepad1.left_trigger);

                if(pivotPosition > 0.42){
                    pivotPosition = 0.42;
                }
                turret.setServoPosition(pivotPosition);
            }


            telemetry.addData("Trigger position", turret.getServoPosition());

            telemetry.update();
        }

    }





}

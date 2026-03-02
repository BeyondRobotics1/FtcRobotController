package org.firstinspires.ftc.teamcode.decode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.Subsystems.DriveTrain;

@TeleOp(name = "Drive to Line test", group = "Decode Test")
public class DriveTrainToLineTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this, false);

        telemetry.addData("gamepad1.dpad_down", "Drive to line" );

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            if(gamepad1.dpad_down)
            {
                driveTrain.driveToLine();

            }
            else {
                driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            telemetry.update();
        }
    }

}

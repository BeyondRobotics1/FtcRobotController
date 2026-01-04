package org.firstinspires.ftc.teamcode.decode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Trigger;

@TeleOp(name = "Lift Test", group = "Decode Test")

public class LiftTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Lift lift = new Lift(hardwareMap);


        telemetry.addData("gamepad1.a", "Engage clutches." );
        telemetry.addData("gamepad1.b", "Disengage clutches." );
        telemetry.addData("gamepad1.x", "Hold Slides." );
        telemetry.addData("gamepad1.y", "Release Slides." );
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;


        while (!isStopRequested() && opModeIsActive()) {

            if(gamepad1.a)
                lift.engageClutch(true);
            else if (gamepad1.b)
                lift.engageClutch(false);
            else if (gamepad1.x){
                lift.releaseHolder(false);
            }
            else if (gamepad1.y){
                lift.releaseHolder(true);
            }

            lift.liftUp(Math.abs(gamepad1.left_stick_y));

            telemetry.addData("Left clutch servo position", lift.getLeftClutchServoPosition());
            telemetry.addData("Right clutch servo position", lift.getRightClutchServoPosition());
            telemetry.addData("Left slide servo position", lift.getLeftSlideServoPosition());
            telemetry.addData("Right slide servo position", lift.getRightSlideServoPosition());
            //trigger.setPosition(pivotPosition)

            telemetry.update();
        }

    }
}

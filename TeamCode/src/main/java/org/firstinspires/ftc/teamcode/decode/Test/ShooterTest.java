package org.firstinspires.ftc.teamcode.decode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Shooter;

@TeleOp(name = "Shooter Test", group = "Decode Test")
public class ShooterTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException
    {
        Shooter shooter = new Shooter(hardwareMap, this);

        telemetry.addData("gamepad1.left_trigger", "shooter");

        waitForStart();

        if(isStopRequested()) return;

        while(!isStopRequested() && opModeIsActive())
        {
            shooter.setPower(gamepad1.left_trigger);

            telemetry.addData("Power", gamepad1.left_trigger);
            telemetry.update();
        }
    }
}

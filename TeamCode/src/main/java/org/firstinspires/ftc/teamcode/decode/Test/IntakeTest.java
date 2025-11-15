package org.firstinspires.ftc.teamcode.decode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.Subsystems.Intake;

@TeleOp(name = "Intake Test", group = "Decode Test")
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException
    {
        Intake intake = new Intake(hardwareMap, this);

        telemetry.addData("left_stick_y up", "outtake");
        telemetry.addData("left_stick_y up", "intake");

        waitForStart();

        if(isStopRequested()) return;

        while(!isStopRequested() && opModeIsActive())
        {
            double intakePower = -gamepad1.left_stick_y;
            if(intakePower >= 0)
            {
                intake.intake(intakePower);
            } else
            {
                intake.outtake(intakePower);
            }

            telemetry.addData("Power", intakePower);
            telemetry.update();
        }
    }
}

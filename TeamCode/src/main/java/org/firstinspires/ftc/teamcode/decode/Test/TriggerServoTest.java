package org.firstinspires.ftc.teamcode.decode.Test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Trigger Servo Test", group = "Decode Test")

public class TriggerServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Servo trigger = hardwareMap.get(Servo.class, "Trigger");

        telemetry.addData("gamepad1.left_trigger", "servo position." );

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            double pivotPosition = gamepad1.left_trigger;
            telemetry.addData("Trigger position", Math.abs(pivotPosition));
            trigger.setPosition(pivotPosition);

            telemetry.update();
        }

    }
}

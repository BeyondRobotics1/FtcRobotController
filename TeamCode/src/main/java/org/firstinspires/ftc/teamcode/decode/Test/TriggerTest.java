package org.firstinspires.ftc.teamcode.decode.Test;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.Subsystems.Trigger;

@Disabled
@TeleOp(name = "Trigger Test", group = "Decode Test")

public class TriggerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("gamepad1.x", "Pollen Servo." );
        telemetry.addData("gamepad1.y", "Nectar Servo." );

        telemetry.addData("gamepad1.a", "Open." );
        telemetry.addData("gamepad1.b", "Close" );
        telemetry.addData("gamepad1.left_trigger", "Set servo position" );

        Trigger trigger = new Trigger(hardwareMap);



        waitForStart();

        if (isStopRequested()) return;

        //trigger.setPosition(0.5);

        while (!isStopRequested() && opModeIsActive()) {

            if(gamepad1.a)
                trigger.open();
            else if (gamepad1.b)
                trigger.close();
            else {

                double pivotPosition = Math.abs(gamepad1.left_trigger);

                telemetry.addData("Trigger position", pivotPosition);
                trigger.setPosition(pivotPosition);
            }



            telemetry.update();
        }

    }
}

package org.firstinspires.ftc.teamcode.biobuzz.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Trigger;

//@Disabled

@TeleOp(name = "Trigger Test", group = "BioBuzz Test")
public class TriggerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Trigger pollenTrigger = null;
        Trigger nectarTrigger = null;


        telemetry.addData("gamepad1.x", "Pollen Trigger" );
        telemetry.addData("gamepad1.y", "Nectar Trigger" );
        telemetry.addData("gamepad1.a", "Open." );
        telemetry.addData("gamepad1.b", "Close" );

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            if (gamepad1.x && pollenTrigger == null) {
                pollenTrigger = new Trigger(hardwareMap, "pollenTrigger",
                        Trigger.kPollenTriggerOpenPosition,
                        Trigger.kPollenTriggerClosePosition, false);

                nectarTrigger = null;
            }
            else if (gamepad1.y && nectarTrigger == null) {
                nectarTrigger = new Trigger(hardwareMap, "nectarTrigger",
                        Trigger.kNectarTriggerOpenPosition,
                        Trigger.kNectarTriggerClosePosition, true);

                pollenTrigger = null;
            }

            if(pollenTrigger != null) {
                if (gamepad1.a)
                    pollenTrigger.open();
                else if (gamepad1.b)
                    pollenTrigger.close();
                else {

                    double pivotPosition = Math.abs(gamepad1.left_trigger);

                    telemetry.addData("Pollen Trigger position", pivotPosition);
                    pollenTrigger.setServoPosition(pivotPosition);
                }
            }
            else if(nectarTrigger != null) {
                if (gamepad1.a)
                    nectarTrigger.open();
                else if (gamepad1.b)
                    nectarTrigger.close();
                else {

                    double pivotPosition = Math.abs(gamepad1.left_trigger);

                    telemetry.addData("Nectar Trigger position", pivotPosition);
                    nectarTrigger.setServoPosition(pivotPosition);
                }
            }



            telemetry.update();
        }

    }
}

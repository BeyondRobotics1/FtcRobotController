package org.firstinspires.ftc.teamcode.decode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.Subsystems.Indexer;

@TeleOp(name = "Indexer Test", group = "Decode Test")

public class IndexerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Indexer indexer = new Indexer(hardwareMap, this);

        indexer.reset();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            //double pivotPosition = Math.abs(gamepad1.left_trigger);

            if(gamepad1.a)
                indexer.queue(1);
            else if (gamepad1.b)
                indexer.queue(2);
            else if (gamepad1.x)
                indexer.reset();
            else
            {
                double pivotPosition = Math.abs(gamepad1.left_trigger);

                telemetry.addData("Trigger position", pivotPosition);
                indexer.setPosition(pivotPosition);
            }

            telemetry.addData("Indexer position", indexer.getPosition());
            telemetry.update();

        }

    }
}

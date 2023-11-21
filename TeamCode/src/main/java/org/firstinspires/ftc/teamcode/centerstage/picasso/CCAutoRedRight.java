package org.firstinspires.ftc.teamcode.centerstage.picasso;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Right", group="Picasso")
//@Disabled

public class CCAutoRedRight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //
        telemetry.addLine("Initializing drive train");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {

            //detect location
            //telemetry.addLine(String.format("\n\nLocation = %d", location));
            telemetry.update();

            sleep(20);
        }

        waitForStart();

        if (isStopRequested()) return;


    }
}

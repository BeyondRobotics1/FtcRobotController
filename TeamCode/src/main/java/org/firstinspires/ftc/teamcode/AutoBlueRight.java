package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Right Side", group="Linear Opmode")
public class AutoBlueRight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //drive train
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        driveTrain.runWithEncoder();

        SleeveDetector sleeveDetector = new SleeveDetector(hardwareMap, this);
        int location = 2;

        while (!isStarted() && !isStopRequested()) {

            // Arm arm = new Arm(hardwareMap);
            location = sleeveDetector.detectPosition();

            telemetry.addLine(String.format("\nlocation=%d", location));
            telemetry.update();

            sleep(20);
        }

        waitForStart();

        if (isStopRequested()) return;


        if (location == 1) {
            driveTrain.moveLeft(29, 0.9);
            sleep(200);
            driveTrain.moveForward(-36, 0.9);
        } else if (location == 2) {
            driveTrain.moveForward(-36, 0.9);
        } else {
            driveTrain.moveLeft(-25, 0.9);
            sleep(200);
            driveTrain.moveForward(-36, 0.9);
        }
    }
}
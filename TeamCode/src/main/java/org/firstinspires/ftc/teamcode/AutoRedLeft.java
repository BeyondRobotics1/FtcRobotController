package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Left Side", group="Linear Opmode")
public class AutoRedLeft extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //drive train
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        driveTrain.runWithEncoder();

        Arm arm = new Arm(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        driveTrain.moveLeft(27, 0.9);


        // driveTrain.moveForward(10, 0.9);

    }
}
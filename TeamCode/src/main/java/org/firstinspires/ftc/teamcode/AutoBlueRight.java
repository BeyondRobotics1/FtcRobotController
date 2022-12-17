package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name="Blue Right Side", group="Linear Opmode")
public class AutoBlueRight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //drive train
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        driveTrain.runWithEncoder();
        //reset drive train's yaw angle
        driveTrain.resetYaw();

        //arm hardware
        Arm arm = new Arm(hardwareMap);
        arm.slideRunWithEncorder();
        Claw claw = new Claw(hardwareMap, this);

        //April tag detector
        SleeveDetector sleeveDetector = new SleeveDetector(hardwareMap, this);
        int location = 2;

        while (!isStarted() && !isStopRequested()) {

            // Arm arm = new Arm(hardwareMap);
            location = sleeveDetector.detectPosition();

            telemetry.addLine(String.format("\n\nLocation = %d", location));
            telemetry.update();

            sleep(20);
        }

        waitForStart();

        if (isStopRequested()) return;

        /*
        arm.moveTo(14.5, 1);
        sleep(5000);
        arm.moveTo(24.5, 1);
        sleep(5000);
        arm.moveTo(34.5, 1);
        sleep(5000);
        arm.moveTo(0, 1);
        */
/*
        if (location == 1) {
            driveTrain.moveLeft(30, 0.4);
            sleep(100);
            driveTrain.moveForward(36, 0.4);
        } else if (location == 2) {
            driveTrain.moveLeft(5, 0.4);
            sleep(100);
            driveTrain.moveForward(36, 0.4);
        } else {
            driveTrain.moveLeft(-28, 0.4);
            sleep(100);
            driveTrain.moveLeft(2, 0.4);
            sleep(100);
            driveTrain.moveForward(36, 0.4);
        }
 */
        //grab the cone
        claw.close();
        sleep(100);
        arm.moveTo(5.5, 1);
        //driveTrain.moveForwardWithGyro(61, 0.6);
        driveTrain.moveForward(61, 0.6);
        sleep(100);
        driveTrain.moveLeft(6, 0.6);
        arm.moveTo(34.5, 1);
        sleep(50);
        arm.setTurretPosition(0);
        sleep(900);
        claw.open();
        sleep(100);
        arm.setTurretPosition(1);
        sleep(450);
        arm.moveTo(6, 1);
        driveTrain.moveLeft(-4, 0.6);
        sleep(50);
        driveTrain.moveForward(-11, 0.6);
        sleep(100);
        //arm.setTurretPosition(1);
        //sleep(500);

        //Move to area
        if(location == 1) {
            driveTrain.moveLeft(23, 0.6);
        }
        else if(location == 3){
            driveTrain.moveLeft(-25,0.6);
        }
        else{

        }


    }
}
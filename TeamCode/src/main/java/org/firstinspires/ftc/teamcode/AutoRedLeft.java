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
        //reset drive train's yaw angle
        driveTrain.resetYaw();

        //arm hardware
        //Arm arm = new Arm(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        Turret turret = new Turret(hardwareMap);
        slide.slideRunWithEncorder();
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


        /*if (location == 1) {
            driveTrain.moveLeft(33, 0.4);
            sleep(200);
            driveTrain.moveLeft(-3,0.4);
            sleep(100);
            driveTrain.moveForward(36, 0.4);
        } else if (location == 2) {
            driveTrain.moveLeft(4, 0.4);
            sleep(100);
            driveTrain.moveForward(36, 0.4);
        } else {
            driveTrain.moveLeft(-25, 0.4);
            sleep(200);
            driveTrain.moveForward(36, 0.4);
        }*/
        //grab the cone
        claw.close();
        sleep(100);
        slide.moveTo(5.5, 1);//Move slide up by 5.5 inches
        //driveTrain.moveForwardWithGyro(61, 0.6);
        driveTrain.moveForward(62, 0.6);//Move to (4,3) high junction
        sleep(100);
        driveTrain.moveLeft(-4, 0.6);//Move closer to the junction
        slide.moveTo(34, 1);//Move 34 inches up to be taller than the high junction
        sleep(50);
        turret.setPosition(2);//turn turret right so cone is on top of junction
        sleep(900);
        claw.open();//release cone to go into junction
        sleep(100);
        turret.setPosition(1);//Turret goes back to the middle
        sleep(450);
        slide.moveTo(6, 1);//turret goes up for 6 inches
        driveTrain.moveLeft(4, 0.6);//Move left to not hit the junction when going to area
        sleep(50);
        driveTrain.moveForward(-11, 0.6);//go back to be prepared to go to area
        sleep(100);
        //arm.setTurretPosition(1);
        //sleep(500);

        //Move to area
        if (location == 1) {
            driveTrain.moveLeft(26, 0.6);//Go to area 1
        } else if (location == 3) {
            driveTrain.moveLeft(-25, 0.6);//Go to area 3
        } else {
        }
    }
}
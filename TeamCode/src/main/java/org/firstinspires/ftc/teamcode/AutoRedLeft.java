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
        Slide slide = new Slide(hardwareMap, this);
        Turret turret = new Turret(hardwareMap, slide);
        slide.runWithEncoder();
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
        //grab the cone
        claw.close();
        sleep(100);
        slide.moveTo(5.5, 1);//Move slide up by 5.5 inches
        driveTrain.moveForwardRamp(56, 0.1, 1.0, 1.25);//Move to (4,3) high junction
        sleep(100);

        double distanceToPole = driveTrain.moveToPole(true,1, 0.3);
        //telemetry.addData("current distance", distanceToPole);
        //telemetry.update();
        driveTrain.moveLeft(1.5 - distanceToPole, 0.5);//Move to 1.5 inches away from the junction
        sleep(100);//100


        //slide.moveToWithoutWaiting(33.6, 1);//Move 34 inches up to be taller than the high junction
        slide.moveTo(33.8, 1);//Move 34 inches up to be taller than the high junction
        turret.setPosition(2);//turn turret right so cone is on top of junction
        sleep(800);//800 ms
        claw.open();//release cone to go into junction
        sleep(100);

        turret.setPosition(1);//Turret goes back to the middle
        sleep(250);//300
        slide.moveTo(5.2, 1);//move the slide down to 5.5 inches
        driveTrain.moveLeft(6, 0.5);//Move left 6 inches to not hit the junction when going to area
        sleep(100);
        driveTrain.moveForward(-11, 0.5);//go back 11.5 to be prepared to turn left
        sleep(100);


        driveTrain.turnToGyroHeading(90, 0.5); //turn left 90 degrees
        sleep(100);
        distanceToPole = driveTrain.moveToPole(true,1,0.3);
        telemetry.addData("current distance", distanceToPole);
        telemetry.update();
        driveTrain.moveLeft(-5.5 - distanceToPole , 0.5);//Move away from the junction to the middle of the tile
        sleep(50);//100
        driveTrain.moveForward(13.5, 0.5);
        sleep(50);
        claw.close();
        sleep( 150);
        slide.moveTo(14, 1);
        driveTrain.moveForward(-30, 0.5);
        sleep(100);

        distanceToPole = driveTrain.moveToPole(true,1, -0.3);
        //telemetry.addData("current distance", distanceToPole);
        //telemetry.update();
        driveTrain.moveLeft(-12.5 - distanceToPole, 0.5);//Move closer to the junction
        sleep(100);//100

        slide.moveTo(33.8, 1);//Move 34 inches up to be taller than the high junction
        turret.setPosition(2);//turn turret right so cone is on top of junction
        sleep(800);//800 ms
        claw.open();//release cone to go into junction
        sleep(100);
        turret.setPosition(1);
        sleep(300); //300 ms
        slide.moveTo(0,1);
        driveTrain.moveLeft(5,0.5);
        sleep(100);

        //Move to area
        if(location == 3) {
            driveTrain.moveForward(-11, 0.6);//Go to area 3
        }
        else if(location == 1){
            driveTrain.moveForward(33.5,0.6);//Go to area 1
        }
        else{
            driveTrain.moveForward(11,0.6);
        }


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
       /* claw.close();
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
        }*/
    }
}
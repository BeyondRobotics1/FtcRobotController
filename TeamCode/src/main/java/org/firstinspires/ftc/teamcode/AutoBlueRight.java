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
        //reset drive train's yaw angle
        driveTrain.resetYaw();

        //arm hardware
        //Arm arm = new Arm(hardwareMap);
        Slide slide = new Slide(hardwareMap);
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
        //driveTrain.moveForwardWithGyro(61, 0.6);
        driveTrain.moveForward(55, 0.6);//Move to (4,3) high junction
        sleep(100);

        double distanceToPole = driveTrain.moveToPole(true,1, 0.3);
        telemetry.addData("current distance", distanceToPole);
        telemetry.update();
        double distanceToMove = distanceToPole - 1.6;
        driveTrain.moveLeft(distanceToMove, 0.6);//Move closer to the junction
        sleep(100);

        slide.moveTo(33.6, 1);//Move 34 inches up to be taller than the high junction
        sleep(50);
        turret.setPosition(0);//turn turret left so cone is on top of junction
        sleep(900);
        claw.open();//release cone to go into junction
        sleep(100);
        turret.setPosition(1);//Turret goes back to the middle
        sleep(450);
        slide.moveTo(6, 1);//move the slide up 6 inches
        driveTrain.moveLeft(-4, 0.6);//Move right to not hit the junction when going to area
        sleep(50);

        slide.moveTo(11.5,1);
        sleep(100);
        driveTrain.moveForward(-11, 0.6);//go back to be prepared to go to area
        sleep(100);
        driveTrain.turnToGyroHeading(-90, 0.5);
        sleep(100);
        driveTrain.moveToPole(false,1,0.3);
        sleep(100);
        driveTrain.moveForward(12.5, 0.3);
        sleep(100);
        claw.close();
        sleep(100);
        slide.moveTo(12, 1);
        sleep(100);
        driveTrain.moveForward(-14.5,0.3);
        sleep(100);
        driveTrain.turnToGyroHeading(-90,1);
        sleep(100);
        claw.open();
        sleep(100);

        //arm.setTurretPosition(1);
        //sleep(500);
//
//        //Move to area
//        if(location == 1) {
//            driveTrain.moveLeft(23, 0.6);//Go to area 1
//        }
//        else if(location == 3){
//            driveTrain.moveLeft(-27,0.6);//Go to area 3
//        }
//        else{
//
//        }


    }
}
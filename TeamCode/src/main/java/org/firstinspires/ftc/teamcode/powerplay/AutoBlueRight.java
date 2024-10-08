package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Right Side", group="Linear Opmode")
@Disabled
public class AutoBlueRight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //
        telemetry.addLine("Initializing drive train");
        telemetry.update();

        //drive train
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this, true);
        driveTrain.resetAndRunUsingEncoder();
        //reset drive train's yaw angle
        driveTrain.resetYaw();

        //
        telemetry.addLine("Initializing slide, turret, and claw");
        telemetry.update();

        //Our robot
        Slide slide = new Slide(hardwareMap, this);
        slide.runWithEncoder();

        Turret turret = new Turret(hardwareMap, slide);
        turret.setToCenterPosition();

        Claw claw = new Claw(hardwareMap, this);
        Aligner aligner = new Aligner(hardwareMap, this);

        //April tag detector
        telemetry.addLine("Initializing camera");
        telemetry.update();

        SleeveDetector sleeveDetector = new SleeveDetector(hardwareMap, this);
        int location = 3; //cone in zone 3

        while (!isStarted() && !isStopRequested()) {

            // Arm arm = new Arm(hardwareMap);
            location = sleeveDetector.detectPosition();

            telemetry.addLine(String.format("\n\nLocation = %d", location));
            telemetry.update();

            sleep(20);
        }

        waitForStart();


        //Log log = new Log("AutoBlueRight"+(int)(Math.random()*1000), true);

        if (isStopRequested()) return;

        //grab the cone
        claw.close();
        sleep(100);

        slide.moveTo(7.5, 1);//Move slide up by 7.5 inches, ready to turn the turret
        turret.setPosition(0);//turn turret left so cone is on top of junction
        slide.moveToWithoutWaiting(24.5, 1);//Move slide up to 24.5 inches
        driveTrain.moveForwardRamp(33, 0.1, 0.8, 1.25);//Move to (4,3) high junction, 1.25
        sleep(100);
        aligner.moveDown();

        double distanceToPole = driveTrain.moveToPole(true,1, 0.3);
        driveTrain.moveLeft(2.5, 0.5);//Move closer distanceToPole - 3.5 to the junction
        sleep(100);//100

        slide.moveTo(20, 0.8);//Move slide down to 18. in. height
        driveTrain.moveLeft(-2, 0.5);//Move away distanceToPole - 3.5 to the junction
        claw.open();
        aligner.moveUp();
        driveTrain.moveLeft(-3.5, 0.5);//Move closer distanceToPole - 3.5 to the junction
        turret.setPosition(1);//turn turret left so cone is on top of junction
        sleep(30);//100

        driveTrain.moveForward(18, 0.8); //push the signal cone away 18
        sleep(30);//100
        driveTrain.moveForward(-8, 0.8);//back -8
        sleep(100);//100

        slide.moveToWithoutWaiting(5, 0.8);//move the slide down to 5.2 inches
        driveTrain.turnToGyroHeading(-90, 0.5); //turn right 90 degrees
        sleep(100);

        //move to pole (5W, 2)
        distanceToPole = driveTrain.moveToPole(false,1,0.3);
        sleep(50);//100

        double distanceToMove = 5.8 - distanceToPole;//move to centre on blue line
        if(Math.abs(distanceToMove) >= 0.4) {
            driveTrain.moveLeft(distanceToMove, 0.5);//Move away 5.5 from the junction to the middle of the tile
            sleep(50);
        }

        int coneToDo = 4;

        //Cone stack, two cones for now
        for(int i = 0; i < 4; i++) {
            driveTrain.moveForward(Slide.moveFromPole[i], 0.6);//speed 0.5
            sleep(25);//50
            claw.close();
            sleep(110);//150
            slide.moveTo(Slide.coneLiftHeights[i], 1);//11

            //move back to the pole
            turret.setPosition(2);//turn turret right
            slide.moveToWithoutWaiting(14.75, 0.8);//move the slide up on top of low junction 14.5
            distanceToPole = driveTrain.moveToPole(false, 1, -0.3);
            sleep(40);//50

            //if(i == 0)
                distanceToMove = distanceToPole - 2.5;//move more to the pole for the first cone
            //else
            //    distanceToMove = distanceToPole - 3;//3.5

            driveTrain.moveLeft(-distanceToMove, 0.5);//Move closer to the junction -4.2
            sleep(40);//50

            //slide down a little to make sure cone is in the pole
            slide.moveTo(12.1, 1);//12.2
            claw.open();
            sleep(40);//50
            driveTrain.moveLeft(distanceToMove-0.4, 0.5);//Move away from the junction distanceToMove-0.4
            turret.setPosition(1);//turn turret to the center so cone is on top of junction
            sleep(325);//350
            slide.moveToWithoutWaiting(Slide.coneStackHeights[i+1], 0.8);//move the slide down to 3.8 inches

            //log.addData(distanceToPole);
            //log.update();
        }


        slide.moveToWithoutWaiting(0, 1);

        driveTrain.moveLeft(-1, 0.5);//1
        sleep(20);
        if(location == 3)
            driveTrain.moveForward(12 , 0.9);//
        else if (location == 2)
            driveTrain.moveForward(-10, 0.9);//
        else {
            driveTrain.fineTuneToGyroHeading(-90, 0.2, 300);
            sleep(40);
            driveTrain.moveForward(-33, 0.9);//
        }

        //make sure using up all 30 seconds of auto time
        //so our slide is fully reset
        sleep(8000);

        //log.close();



//        log.addData(distanceToPole);
//        log.update();
//
//        log.close();

//        //grab the cone
//        claw.close();
//        sleep(100);
//        slide.moveTo(5.5, 1);//Move slide up by 5.5 inches
//        driveTrain.moveForwardRamp(56, 0.1, 1.0, 1.25);//Move to (4,3) high junction
//        sleep(100);
//
//        double distanceToPole = driveTrain.moveToPole(true,1, 0.3);
//        //telemetry.addData("current distance", distanceToPole);
//        //telemetry.update();
//        driveTrain.moveForward(-0.5, 0.4);
//        driveTrain.moveLeft(distanceToPole - 2.2, 0.5);//Move closer 2 to the junction
//        sleep(100);//100
//
//
//        //slide.moveToWithoutWaiting(33.6, 1);//Move 34 inches up to be taller than the high junction
//        slide.moveTo(33.5, 1);//Move 33.7 inches up to be taller than the high junction
//        turret.setPosition(0);//turn turret left so cone is on top of junction
//        sleep(800);//800
//        claw.open();//release cone to go into junction
//        sleep(100);
//
//        driveTrain.moveLeft(-4.5, 0.5);//Move right 4.5 inches to not hit the junction when going to area
//        sleep(100);//100
//
//        turret.setPosition(1);//Turret goes back to the middle
//        sleep(250);//300
//        slide.moveTo(5.2, 1);//move the slide down to 5.5 inches
//

//        driveTrain.moveForward(-12, 0.5);//go back 12 to be prepared to turn right
//        sleep(100);

//        //slide.moveToWithoutWaiting(33.6, 1);//Move 34 inches up to be taller than the high junction
//        slide.moveTo(33.5, 1);//Move 33.7 inches up to be taller than the high junction
//        turret.setPosition(0);//turn turret left so cone is on top of junction
//        sleep(800);//800
//        slide.moveTo(31, 1);//Move 33.7 inches up to be taller than the high junction

//        claw.open();//release cone to go into junction
//        sleep(100);
//
//        turret.setPosition(1);//Turret goes back to the middle
//        sleep(250);//300
//        slide.moveTo(5.2, 1);//move the slide down to 5.5 inches
//
//        driveTrain.moveLeft(-4.5, 0.5);//Move right 4.5 inches to not hit the junction when going to area
//        sleep(100);//100
//        driveTrain.moveForward(-12, 0.5);//go back 12 to be prepared to turn right
//        sleep(100);
//
//        driveTrain.turnToGyroHeading(-90, 0.5); //turn right 90 degrees
//        sleep(100);
//
//
//        //move to pole (5, 2)
//        distanceToPole = driveTrain.moveToPole(false,1,0.3);
//        sleep(100);
//        double distanceToMove = 5.8 - distanceToPole;//move to blue line
//        driveTrain.moveLeft(distanceToMove, 0.5);//Move away 5.5 from the junction to the middle of the tile
//        sleep(50);
//
//
//        //move to cone stack
//        driveTrain.moveForward(13.4, 0.5);//13.5
//        sleep(50);
//        claw.close();
//        sleep(150);
//        slide.moveTo(11, 1);//11
//
//        driveTrain.moveForwardRamp(-33.5, 0.1, 0.9, 1.25);//-33
//        sleep(50);
//        distanceToPole = driveTrain.squareToPoles(1, -0.1, 1300);//left distance to pole
//        sleep(50);
//        driveTrain.moveForward(-1, 0.4);//-1.5
//        sleep(50);
//        driveTrain.moveLeft(distanceToPole - 3.7, 0.5);//Move closer to the junction - 3.5
//
//        sleep(50);
//        slide.moveTo(33.7, 1);//Move 34 inches up to be taller than the high junction
//        turret.setPosition(0);//turn turret left so cone is on top of junction
//        sleep(800);//800
//        slide.moveTo(31.2, 1);//Move 31 inches up to be taller than the high junction
//
//        claw.open();//release cone to go into junction
//        sleep(100);
//        driveTrain.moveLeft(-4,0.5);//3
//        sleep(100);
//        turret.setPosition(1);
//        sleep(250);
//        slide.moveTo(0,1);

//
//        //Move to area
//        if(location == 1) {
//            driveTrain.moveForward(-11, 0.6);//Go to area 1
//        }
//        else if(location == 3){
//            driveTrain.moveForward(33.5,0.6);//Go to area 3
//        }
//        else{
//            driveTrain.moveForward(11,0.6);
//        }




    }
}
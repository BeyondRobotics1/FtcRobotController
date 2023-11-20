package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Left Side", group="Linear Opmode")
@Disabled
public class AutoRedLeft extends LinearOpMode {

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
        int location = 1;

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

        slide.moveTo(7.5, 1);//Move slide up by 8 inches, ready to turn the turret
        turret.setPosition(2);//turn turret right so cone is on top of junction
        slide.moveToWithoutWaiting(24.5, 1);//Move slide up by 5.5 inches
        driveTrain.moveForwardRamp(33, 0.1, 0.8, 1.25);//Move to (2,2) high junction
        sleep(100);//100
        aligner.moveDown();

        double distanceToPole = driveTrain.moveToPole(false,1, 0.3);
        sleep(30);
        driveTrain.moveForward(0.5,0.5);
        sleep(30);
        driveTrain.moveLeft(-3.5,0.5);//Move closer distanceToPole -3.5 to the junction
        sleep(100);//100

        slide.moveTo(20, 0.8);//Move slide down by 2.5 inches (21.5)
        driveTrain.moveLeft(2.5, 0.5);//Move closer 2.5 to the junction
        claw.open();
        aligner.moveUp();
        driveTrain.moveLeft(3.5, 0.5);//Move closer 3.5 to the junction
        turret.setPosition(1);//turn turret left so cone is on top of junction
        sleep(30);//100

        driveTrain.moveForward(19, 0.9); //push the signal cone away 20
        sleep(30);//100
        driveTrain.moveForward(-9, 0.8); //originally -9 inches
        sleep(100);//100

        slide.moveToWithoutWaiting(5, 0.8);//move the slide down to 5.2 inches
        driveTrain.turnToGyroHeading(90, 0.5); //turn right 90 degrees
        sleep(100);

        //move to pole (1, 2)
        distanceToPole = driveTrain.moveToPole(true,1,0.3);
        sleep(50);//100
        double distanceToMove = distanceToPole - 7;//from robot cent to pole 7
        if(Math.abs(distanceToMove) >= 0.4){
            driveTrain.moveLeft(distanceToMove, 0.5);//Move away 5.5 from the junction to the middle of the tile
            sleep(50);
        }

        //Cone stack, two cones for now
        for(int i = 0; i < 4; i++) {
            driveTrain.moveForward(Slide.moveFromPole[i], 0.6);//slide.moveFromPole[i]+0.2, 0.5
            sleep(25);//50
            claw.close();
            sleep(110);//150
            slide.moveTo(Slide.coneLiftHeights[i], 1);//11

            //move back to the pole
            turret.setPosition(0);//turn turret left
            slide.moveToWithoutWaiting(14.75, 0.8);//move the slide up on top of low junction 14.5
            distanceToPole = driveTrain.moveToPole(true, 1, -0.3);
            sleep(40);

            distanceToMove = distanceToPole - 2.5;// original is 3

            driveTrain.moveLeft(distanceToMove, 0.5);//Move left closer to the junction -4.2
            sleep(40);

            //slide down a little to make sure cone is in the pole
            slide.moveTo(12.1, 1);//12.2
            claw.open();
            sleep(40);
            driveTrain.moveLeft(-distanceToMove + 0.1, 0.5);//Move right away from the junction
            turret.setPosition(1);//turn turret to the center so cone is on top of junction
            sleep(325);//350
            slide.moveToWithoutWaiting(Slide.coneStackHeights[i+1], 0.8);//move the slide down to 3.8 inches

            //log.addData(distanceToPole);
            //log.update();
        }
////
////
        slide.moveToWithoutWaiting(0, 1);

        driveTrain.moveLeft(-1, 0.5);//Move away from the junction

        if(location == 1)
            driveTrain.moveForward(12 , 0.9);//
        else if (location == 2)
            driveTrain.moveForward(-10, 0.9);//
        else {
            driveTrain.fineTuneToGyroHeading(90, 0.2, 300);
            sleep(40);
            driveTrain.moveForward(-32.5, 0.9);//-33, 0.9
        }

        sleep(8000);
//
/*

        //grab the cone
        claw.close();
        sleep(100);
        slide.moveTo(5.5, 1);//Move slide up by 5.5 inches
        driveTrain.moveForwardRamp(56, 0.1, 1.0, 1.25);//Move to (4,3) high junction
        sleep(100);

        double distanceToPole = driveTrain.moveToPole(false,1, 0.3);
        //telemetry.addData("current distance", distanceToPole);
        //telemetry.update();
        //driveTrain.moveForward(-0.4, 0.4);
        sleep(50);//100
        driveTrain.moveLeft(2.1 - distanceToPole, 0.5);//Move closer 2.2 to the junction
        sleep(100);//100


        //slide.moveToWithoutWaiting(33.6, 1);//Move 34 inches up to be taller than the high junction
        slide.moveTo(33.5, 1);//Move 33.7 inches up to be taller than the high junction
        turret.setPosition(2);//turn turret left so cone is on top of junction
        sleep(800);//800
        slide.moveTo(31, 1);//Move 33.7 inches up to be taller than the high junction
        claw.open();//release cone to go into junction
        sleep(100);

        driveTrain.moveLeft(4.5, 0.5);//Move right 4.5 inches to not hit the junction when going to area
        sleep(100);//100

        turret.setPosition(1);//Turret goes back to the middle
        sleep(250);//300

        slide.moveTo(Slide.coneStackHeights[0], 1);//move the slide down to 5.2 inches


        driveTrain.moveForward(-10.5, 0.5);//go back 12 to be prepared to turn right
        sleep(100);

        driveTrain.turnToGyroHeading(91, 0.5); //turn left 90 degrees
        sleep(100);

        //move to pole (1, 2)
        distanceToPole = driveTrain.moveToPole(true,1,0.3);
        sleep(100);
        double distanceToMove = distanceToPole - 7;//from robot cent to pole 7
        if(Math.abs(distanceToMove) > 0.4) {
            driveTrain.moveLeft(distanceToMove, 0.5);//Move away 5.5 from the junction to the middle of the tile
            sleep(50);
        }

        //Cone stack, one cones for now
        for(int i = 0; i < 1; i++) {
            driveTrain.moveForward(slide.moveFromPole[i]+0.2, 0.5);//13.5
            sleep(50);//50
            claw.close();
            sleep(150);
            slide.moveTo(slide.coneLiftHeights[i], 1);//11

            //move back to the pole
            turret.setPosition(0);//turn turret left
            slide.moveToWithoutWaiting(14.5, 0.8);//move the slide up on top of low junction 15
            distanceToPole = driveTrain.moveToPole(true, 1, -0.3);
            sleep(50);

            distanceToMove = distanceToPole - 3;//3.5

            driveTrain.moveLeft(distanceToMove, 0.5);//Move left closer to the junction -4.2
            sleep(50);

            //slide down a little to make sure cone is in the pole
            slide.moveTo(12.2, 1);//12.5
            claw.open();
            sleep(50);
            driveTrain.moveLeft(-distanceToMove + 0.1, 0.5);//Move right away from the junction
            turret.setPosition(1);//turn turret to the center so cone is on top of junction
            sleep(380);
            slide.moveToWithoutWaiting(slide.coneStackHeights[i+1], 0.8);//move the slide down to 3.8 inches

            //log.addData(distanceToPole);
            //log.update();
        }

        //move to cone stack
        driveTrain.moveForward(14.1, 0.5);//13.5
        sleep(50);
        claw.close();
        sleep(150);
        slide.moveTo(11, 1);//14

        //driveTrain.moveForward(-31, 0.5);
        driveTrain.moveForwardRamp(-32, 0.1, 0.9, 1.25);//-30

        sleep(100);
        SquareToPoolResult result = driveTrain.squareToPoles(1, -0.1, 1200);//left distance to pole
        sleep(50);
        driveTrain.moveForward(-1.8, 0.4);

//        if(result.left) //left pole is squared first
//            driveTrain.moveLeft( result.distance - 11.5, 0.5);//Move closer to the junction distanceToPole - 11.5
//        else
//            driveTrain.moveLeft( 2.5 - result.distance, 0.5);//Move closer to the junction 3.5 - result.distance

        sleep(50);
        double leftDistance = driveTrain.getLeftDistanceINCH();
        double rightDistance = driveTrain.getRightDistanceINCH();

        if(rightDistance < 10)
            driveTrain.moveLeft( 2 - rightDistance, 0.5);//Move closer to the junction 3.5 - result.distance
        else if(leftDistance < 10)
            driveTrain.moveLeft( leftDistance - 11.5, 0.5);//Move closer to the junction distanceToPole - 11.5
        else
            driveTrain.moveLeft( -4.2, 0.5);

//        else

        //sleep(50);
        slide.moveTo(33.7, 1);//Move 34 inches up to be taller than the high junction
        turret.setPosition(2);//turn turret left so cone is on top of junction
        sleep(800);//800
        slide.moveTo(31.2, 1);//Move 31 inches up to be taller than the high junction

        claw.open();//release cone to go into junction
        sleep(100);

        driveTrain.moveLeft(4.5, 0.5);//5
        sleep(100);

        turret.setPosition(1);
        sleep(250);//250

        slide.moveToWithoutWaiting(0,1);

        //Move to area
        if(location == 3) {
            driveTrain.moveForward(-11, 0.7);//0.6 Go to area 1
        }
        else if(location == 1){
            driveTrain.moveForward(33.5,0.7);//Go to area 3
        }
        else{
            driveTrain.moveForward(11,0.7);
        }

        sleep((8000));

 */
    }
}


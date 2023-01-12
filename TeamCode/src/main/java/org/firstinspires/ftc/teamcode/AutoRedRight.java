package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Right Side", group="Linear Opmode")
public class AutoRedRight extends LinearOpMode {

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

        //April tag detector
        telemetry.addLine("Initializing camera");
        telemetry.update();

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
        driveTrain.moveForward(-1, 0.4);
        driveTrain.moveLeft(-2.3, 0.5);//Move closer distanceToPole - 2.3 to the junction
        sleep(100);//100


        //slide.moveToWithoutWaiting(33.6, 1);//Move 34 inches up to be taller than the high junction
        slide.moveTo(33.5, 1);//Move 33.7 inches up to be taller than the high junction
        turret.setPosition(0);//turn turret left so cone is on top of junction
        sleep(800);//800
        slide.moveTo(31, 1);//Move 33.7 inches up to be taller than the high junction
        claw.open();//release cone to go into junction
        sleep(100);

        driveTrain.moveLeft(-4.5, 0.5);//Move right 4.5 inches to not hit the junction when going to area
        sleep(100);//100

        turret.setPosition(1);//Turret goes back to the middle
        sleep(250);//300
        slide.moveTo(5.2, 1);//move the slide down to 5.5 inches


        driveTrain.moveForward(-12, 0.5);//go back 12 to be prepared to turn right
        sleep(100);

        driveTrain.turnToGyroHeading(-90, 0.5); //turn right 90 degrees
        sleep(100);


        //move to pole (5, 2)
        distanceToPole = driveTrain.moveToPole(false,1,0.3);
        sleep(100);
        double distanceToMove = 5.8 - distanceToPole;//move to blue line
        driveTrain.moveLeft(distanceToMove, 0.5);//Move away 5.5 from the junction to the middle of the tile
        sleep(50);


        //move to cone stack
        driveTrain.moveForward(13.4, 0.5);//13.5
        sleep(50);
        claw.close();
        sleep(150);
        slide.moveTo(11, 1);//11

        driveTrain.moveForwardRamp(-33.5, 0.1, 0.9, 1.25);//-33
        sleep(50);
        SquareToPoolResult result = driveTrain.squareToPoles(1, -0.1, 1300);//left distance to pole
        sleep(50);
        driveTrain.moveForward(-1, 0.4);//-1.5
        sleep(50);

        if(result.left) //left pole is squired
            driveTrain.moveLeft( result.distance - 3.5, 0.5);//Move closer to the junction distanceToPole - 3.7
        else
            driveTrain.moveLeft( 10.5 - result.distance, 0.5);//Move closer to the junction distanceToPole - 11.5

        sleep(50);
        slide.moveTo(33.7, 1);//Move 34 inches up to be taller than the high junction
        turret.setPosition(0);//turn turret left so cone is on top of junction
        sleep(800);//800
        slide.moveTo(31.2, 1);//Move 31 inches up to be taller than the high junction

        claw.open();//release cone to go into junction
        sleep(100);

        driveTrain.moveLeft(-4,0.5);//3
        sleep(100);

        turret.setPosition(1);
        sleep(250);
        slide.moveTo(0,1);


        //Move to area
        if(location == 3) {
            driveTrain.moveForward(-11, 0.6);//Go to area 1
        }
        else if(location == 1){
            driveTrain.moveForward(33.5,0.6);//Go to area 3
        }
        else{
            driveTrain.moveForward(11,0.6);
        }


    }
}
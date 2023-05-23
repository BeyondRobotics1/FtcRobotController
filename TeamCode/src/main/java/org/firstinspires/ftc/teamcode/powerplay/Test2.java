package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Test2", group="Linear Opmode")
@Disabled
public class Test2 extends LinearOpMode {

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

        //grab the cone
        claw.close();
        sleep(100);

        waitForStart();

        if (isStopRequested()) return;

//        //slide.moveTo(5.5, 1);//Move slide up by 5.5 inches
//        slide.moveToWithoutWaiting(5.5, 0.8);//Move slide up by 5.5 inches
//        driveTrain.moveForwardRamp(56, 0.1, 1.0, 1.25);//Move to (4,3) high junction
//        sleep(100);
//
//        turret.setPosition(0);//turn turret left so cone is on top of junction
//        slide.moveToWithoutWaiting(33.5, 0.8);//Move slide up by 5.5 inches
//
//        double distanceToPole = driveTrain.moveToPole(true,1, 0.3);
//        //telemetry.addData("current distance", distanceToPole);
//        //telemetry.update();
//        driveTrain.moveForward(-0.5, 0.4);
//        driveTrain.moveLeft(distanceToPole - 2.2, 0.5);//Move closer 2 to the junction
//        sleep(9000);//100

        driveTrain.moveLeft(4, 0.5);
        sleep(50);
        driveTrain.moveLeft(-4, 0.5);
        sleep(50);
        driveTrain.moveLeft(4, 0.5);
        sleep(50);
        driveTrain.moveLeft(-4, 0.5);
        sleep(50);
        driveTrain.moveLeft(4, 0.5);
        sleep(50);
        driveTrain.moveLeft(-4, 0.5);
        sleep(50);

    }
}
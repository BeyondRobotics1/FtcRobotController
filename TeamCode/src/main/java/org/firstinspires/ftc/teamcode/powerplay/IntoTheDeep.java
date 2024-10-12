package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue/Red Right", group="Linear Opmode")
//@Disabled
public class IntoTheDeep extends LinearOpMode {

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


        waitForStart();




        if (isStopRequested()) return;

        claw.close();


        slide.moveToWithoutWaiting(28, 0.85);

        driveTrain.moveForwardRamp(29.5, 0.2, 0.6, 1.25);
        sleep(50);
        slide.moveTo(20, 0.8);
        sleep(50);
        claw.open();
        driveTrain.moveForwardRamp(-5, 0.2, 0.8, 1.25);
        slide.moveToWithoutWaiting(0, 0.8);

        sleep(50);
        driveTrain.moveLeft(-28, 0.7);
        sleep(50);
        driveTrain.moveForwardRamp(26, 0.2, 0.6, 1.25);
        sleep(50);
        driveTrain.moveLeft(-12, 0.7);
        sleep(50);
        driveTrain.moveForwardRamp(-38, 0.2, 0.6, 1.25);
        sleep(50);
        driveTrain.moveForwardRamp(38, 0.2, 0.7, 1.25);
        sleep(50);
        driveTrain.moveLeft(-11, 0.7);
        sleep(50);
        driveTrain.moveForwardRamp(-39, 0.2, 0.7, 1.25);
        sleep(50);
        driveTrain.moveForwardRamp(39, 0.2, 0.7, 1.25);
        sleep(50);
        driveTrain.moveLeft(-7.7, 0.7);
        sleep(50);
        driveTrain.moveForwardRamp(-47, 0.2, 0.7, 1.25);
        sleep(50);
        driveTrain.moveForwardRamp(1.5,0.2,0.7,1.25);
        sleep(50);
    }
}

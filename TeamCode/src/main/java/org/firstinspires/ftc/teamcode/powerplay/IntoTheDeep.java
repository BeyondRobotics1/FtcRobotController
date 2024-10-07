package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Left", group="Linear Opmode")
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

        driveTrain.moveForwardRamp(29, 0.2, 0.6, 1.25);
        sleep(200);
        slide.moveTo(20, 0.8);
        sleep(300);
        claw.open();
        slide.moveToWithoutWaiting(0, 0.8);
        driveTrain.moveForwardRamp(-27, 0.2, 0.8, 1.25);
        sleep(300);
        driveTrain.moveLeft(-40, 0.7);


    }
}

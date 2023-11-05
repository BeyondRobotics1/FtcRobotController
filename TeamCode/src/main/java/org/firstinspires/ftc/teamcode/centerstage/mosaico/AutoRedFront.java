package org.firstinspires.ftc.teamcode.centerstage.mosaico;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.powerplay.DriveTrain;

@Autonomous(name="Red Front", group="Linear Opmode")
//@Disabled
public class AutoRedRight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //
        telemetry.addLine("Initializing drive train");
        telemetry.update();

        //drive train
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this, true);
        MosaicoClaw claw = new MosaicoClaw(hardwareMap, this);

        //
        telemetry.addLine("Initializing drive and claw");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            sleep(20);
        }

        waitForStart();


        //Log log = new Log("AutoBlueRight"+(int)(Math.random()*1000), true);

        if (isStopRequested()) return;
            driveTrain.moveForward(32.5,0.5);
            driveTrain.moveForward(-5,0.5);
        //make sure using up all 30 seconds of auto time
        //so our slide is fully reset
        sleep(8000);

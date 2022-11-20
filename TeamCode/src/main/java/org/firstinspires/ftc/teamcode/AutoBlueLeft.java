package org.firstinspires.ftc.teamcode;
/**
 * This code is for the Autonomous period for the position Blue Left.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="Blue Left Side", group="Linear Opmode")
public class AutoBlueLeft extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

        //drive train
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        driveTrain.runWithEncoder();

        SleeveDetector sleeveDetector = new SleeveDetector(hardwareMap, this);
        int location = 2;

        while (!isStarted() && !isStopRequested()) {

            // Arm arm = new Arm(hardwareMap);
            location = sleeveDetector.detectPosition();

            telemetry.addLine(String.format("\nlocation=%d", location));
            telemetry.update();

            sleep(20);
        }

        waitForStart();

        if (isStopRequested()) return;


        if(location == 1){
            driveTrain.moveLeft(26, 0.9);
            sleep(200);
            driveTrain.moveForward(-36, 0.9);
        }else if(location == 2){
            driveTrain.moveForward(-36, 0.9);
        }else{
            driveTrain.moveLeft(-20, 0.9);
            sleep(200);
            driveTrain.moveForward(-38, 0.9);
        }

     /*   driveTrain.moveForward(-14, 0.9);
        sleep(200);
        arm.closeClaw();
        arm.moveSlide(-0.9);
        sleep(2000);
        arm.moveSlide(1);
        sleep(1500);
        arm.setClawPosition(1);
        sleep(2000);
        arm.openClaw();
        arm.moveSlide(-0.9);*/


    }
}

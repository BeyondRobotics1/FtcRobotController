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

        Arm arm = new Arm(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        driveTrain.moveForward(-14, 0.9);
        sleep(200);
        arm.closeClaw();
        arm.moveSlide(-0.9);
        sleep(2000);
        arm.moveSlide(1);
        sleep(1500);
        arm.setClawPosition(1);
        sleep(2000);
        arm.openClaw();
        arm.moveSlide(-0.9);


    }
}

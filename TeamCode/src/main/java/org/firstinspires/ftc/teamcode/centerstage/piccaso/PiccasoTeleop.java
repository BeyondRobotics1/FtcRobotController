package org.firstinspires.ftc.teamcode.centerstage.piccaso;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@TeleOp(group = "PbnJ")
//@Disabled;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
public class PiccasoTeleop extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //implement new claw and slide later
//        MosaicoClaw claw = new MosaicoClaw(hardwareMap, this);
//        MosaicoSlide slide = new MosaicoSlide(hardwareMap, this);


        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            drive.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

//            if(gamepad2.right_bumper){
//                claw.close();
//            }
//
//            else {
//                claw.open();
//            }


            //hold left bumper or dpad right button to keep the claw at up position
//            if(gamepad2.left_bumper || gamepad2.dpad_right){
//                claw.setArmPosition(3); //up
//            }
//            else if(gamepad2.dpad_up){
//                claw.setArmPosition(2);
//            }
//            else if(gamepad2.dpad_left){
//                claw.setArmPosition(1);
//            }
//            else
//                claw.setArmPosition(0); //down

//            claw.setArmPosition(1 - Math.abs(gamepad2.right_stick_y));
//
//
//            //release left bumper to keep the claw at the up position
//            slide.setPower(-gamepad2.left_stick_y);
        }
    }
}
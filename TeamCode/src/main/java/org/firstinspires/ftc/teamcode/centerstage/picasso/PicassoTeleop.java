package org.firstinspires.ftc.teamcode.centerstage.picasso;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.powerplay.TeleOp1;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@TeleOp(group = "PbnJ")
//@Disabled;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
public class PicassoTeleop extends LinearOpMode {

    enum SlideOp
    {
        GROUND,
        LOW,
        MEDIUM,
        HIGH,
        SCORE,
        MANUAL
    }

    @Override
    public void runOpMode() throws InterruptedException {


        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initializing drive train");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //IMU imu = drive.imu();

        telemetry.addLine("Initializing intake");
        telemetry.update();
        Intake intake = new Intake(hardwareMap, this);

        telemetry.addLine("Initializing pixel placer");
        telemetry.update();
        PixelPlacer pixelPlacer = new PixelPlacer(hardwareMap, this);
        pixelPlacer.Unlock();

        telemetry.addLine("Initializing arm");
        telemetry.update();
        Arm arm = new Arm(hardwareMap, this);

        telemetry.addLine("Initializing outtake");
        telemetry.update();
        Outtake outtake = new Outtake(hardwareMap, this);

        telemetry.addLine("Initializing slide");
        telemetry.update();
        PicassoSlide slide = new PicassoSlide(hardwareMap, this);
        slide.runWithEncoder();

        telemetry.addLine("Initializing Hanger");
        telemetry.update();
        Hanger hanger = new Hanger(hardwareMap, this);

        telemetry.addLine("Initializing Launcher");
        telemetry.update();
        DroneLauncher launcher = new DroneLauncher(hardwareMap, this);

        //We use this timer to check the game time that has elapsed
        ElapsedTime timer = new ElapsedTime();

        //
        telemetry.addLine("Initialization done, wait for start");
        telemetry.update();

        waitForStart();

        telemetry.addLine("TeleOp started");
        telemetry.update();

        //restart the timer
        timer.reset();

        SlideOp slideOp = SlideOp.MANUAL;
        boolean robotCentric = true;

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            /////drive train
            drive.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            /////Slide control
            //Move slide to specific white strip height
            //left bumper + a dpad key (auto move slide)
            if(gamepad2.left_bumper) {

                if (gamepad2.dpad_down && slideOp != SlideOp.GROUND) {
                    slideOp = SlideOp.GROUND;
                    slide.moveToWhiteStripWithoutWaiting(0, 1); //ground
                }
                else if (gamepad2.dpad_left && slideOp != SlideOp.LOW) {
                    slideOp = SlideOp.LOW;
                    slide.moveToWhiteStripWithoutWaiting(1, 1); //low
                }
                else if (gamepad2.dpad_up && slideOp != SlideOp.MEDIUM){
                    slideOp = SlideOp.MEDIUM;
                    slide.moveToWhiteStripWithoutWaiting(2, 1); //medium//
                }
                else if (gamepad2.dpad_right && slideOp != SlideOp.HIGH) {
                    slideOp = SlideOp.HIGH;
                    slide.moveToWhiteStripWithoutWaiting(3, 1); //high
                }
            }
            else //otherwise left stick y (manual move slide)
            {
                slideOp = SlideOp.MANUAL;
                slide.setPower(-gamepad2.left_stick_y);
            }
            slide.autoMoveToWithoutWaitingLoop();


            //////intake & outtake control
            if(gamepad1.left_bumper){
                //take in
                outtake.TakeIn();
                intake.setPower(0.7);
            }
            else if(gamepad1.right_bumper){
                //spit out
                intake.setPower(-0.7);
            }
            else{
                intake.setPower(0);
            }

            //release the pixel onto the backdrop using right bumper
            if(gamepad2.right_bumper)
                outtake.TakeOut();
            else if(intake.intakeMode() == Intake.IntakeMode.IDLE)
                outtake.Hold();

            //////Arm control
            //controlling the arm using right stick y
            if(gamepad2.right_stick_y < -0.5){
                arm.goUp();
            }
            else {
                //by default, the outtake will be in down position
                arm.goDown();
            }


            //////hanger control
            //use gamepad1 left/right trigger to move the hook servos
            if(gamepad1.left_trigger > 0.1) {
                hanger.HookDown();
            }
            else if (gamepad1.right_trigger > 0.1) {
                hanger.HookUp();
            }
            else
                hanger.HookStay();

            //use gamepad1 X and Y button to rotate the hanger motor
            if(gamepad1.y)
                hanger.SetMotorPower(1);
            else if(gamepad1.x)
                hanger.SetMotorPower(-1);
            else
                hanger.SetMotorPower(0);


            /////Launcher control
            if(gamepad2.y)
                launcher.ReleaseTrigger();

            //////damage control for pixel placer
            //Pixel placer may accidentally lock a pixel during teleop
            if (gamepad1.dpad_down)
                pixelPlacer.Lock();
            else
                pixelPlacer.Unlock();

//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", botHeading * 180 / Math.PI);
//            telemetry.update();
        }
    }
}
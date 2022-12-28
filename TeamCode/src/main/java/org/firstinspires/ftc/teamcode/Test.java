package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.List;

@TeleOp(name="Test", group="Linear Opmode")
public class Test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSlide");

        //drive train
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        driveTrain.runWithEncoder();
        //reset drive train's yaw angle
        driveTrain.resetYaw();

        //arm hardware
        //Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap, this);
        Slide slide = new Slide(hardwareMap);
        slide.slideRunWithEncorder();
        Turret turret = new Turret(hardwareMap, slide);

        //April tag detector
        SleeveDetector sleeveDetector = new SleeveDetector(hardwareMap, this);
        int location = 2;

        // Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        // --------------------------------------------------------------------------------------
        // Run test cycles using AUTO cache mode
        // In this mode, only one bulk read is done per cycle, UNLESS you read a specific encoder/velocity item AGAIN in that cycle.
        // --------------------------------------------------------------------------------------

        // Important Step 3: Option A. Set all Expansion hubs to use the AUTO Bulk Caching mode
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        while (!isStarted() && !isStopRequested()) {

            // Arm arm = new Arm(hardwareMap);
            location = sleeveDetector.detectPosition();

            telemetry.addLine(String.format("\n\nLocation = %d", location));
            telemetry.update();

            sleep(20);
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //slide.moveTo(5, 0.8);
            //sleep(5000);
           // slide.moveTo(20, 0.8);
            //sleep(5000);
            //slide.moveTo(0, 0.8);


            ////use left stick y to set the power slide motors
            //slide.setPower(-gamepad2.left_stick_y);
            //telemetry.addData("Slide height inches", slide.getSlideHeightInches());

            ////color sensor
            //NormalizedRGBA colors = colorSensor.getNormalizedColors();
            //telemetry.addData("Red", colors.red);
            //telemetry.addData("Blue", colors.blue);
            //telemetry.addData("Alpha", colors.alpha);

            //driveTrain.moveToPole(3, 0.6);

            //telemetry.update();

            if(gamepad2.y)
                driveTrain.turnToGyroHeading(90, 0.6);

            if(gamepad2.x)
                driveTrain.turnToGyroHeading(-90, 0.6);

            if(gamepad2.b)
                driveTrain.turnToGyroHeading(0, 0.6);

            if(gamepad2.a)
                driveTrain.turnClockwise(90, 0.6);

            //sleep(50);
        }
    }
}
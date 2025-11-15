package org.firstinspires.ftc.teamcode.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;

@TeleOp(name = "Concept: Fly Wheel test", group = "Concept")
@Disabled
public class FlyWheelTest extends LinearOpMode {

    private GamepadEx gp;
    private MotorGroup flyWheel;
    public static double kP = 20;
    public static double kV = 0.7;

    private DcMotorEx motor;

    @Override
    public void runOpMode() {
        //motorVelocityTest();
        flyWheelTest();
    }

    private void flyWheelTest()
    {
        Motor motor1 = new Motor(hardwareMap, "leftFlywheel", Motor.GoBILDA.BARE);
        Motor motor2 = new Motor(hardwareMap, "rightFlywheel", Motor.GoBILDA.BARE);
        motor1.setInverted(true);

        gp = new GamepadEx(gamepad2);
        flyWheel = new MotorGroup( motor1, motor2);

        flyWheel.setRunMode(Motor.RunMode.VelocityControl);
        flyWheel.setVeloCoefficients(0.8, 0.01, 0);
        flyWheel.setFeedforwardCoefficients(0, 0.8);//0.7

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //same as above
        //hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive())
        {
            hubs.forEach(LynxModule::clearBulkCache);

            if(gp.isDown(GamepadKeys.Button.A)) {
                flyWheel.set(1);
            }
            else
                flyWheel.stopMotor();

            //COUNTS_PER_MOTOR_REV    = 28.0;
            //MOTOR MAX RMP = 6000;
            //ACHIEVABLE_MAX_TICKS_PER_SECOND = 28 * 6000 / 60 = 2800;

            telemetry.addData("ACHIEVABLE_MAX_TICKS_PER_SECOND", motor1.ACHIEVABLE_MAX_TICKS_PER_SECOND);

            // we can obtain a list of corrected velocities with each item in the list
            // representing the motor passed in as an input to the constructor.
            // so, our flywheel_left is index 0 and flywheel_right is index 1
            List<Double> velocities = flyWheel.getVelocities();
            telemetry.addData("Left Flywheel Velocity", velocities.get(0));
            telemetry.addData("Right Flywheel Velocity", velocities.get(1));

            telemetry.addData("Left Flywheel RAW Velocity", motor1.encoder.getRawVelocity());
            telemetry.addData("Left Flywheel power", motor1.motor.getPower());

            gp.readButtons();

            telemetry.update();
        }
    }

    private void motorVelocityTest()
    {
        motor = hardwareMap.get(DcMotorEx.class, "motor1");

        gp = new GamepadEx(gamepad2);

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //same as above
        //hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            hubs.forEach(LynxModule::clearBulkCache);

            if(gp.isDown(GamepadKeys.Button.A)) {
                motor.setVelocity(2800 * 0.9);
            }
            else
                motor.setVelocity(0);

            //COUNTS_PER_MOTOR_REV    = 28.0;
            //MOTOR MAX RMP = 6000;
            //ACHIEVABLE_MAX_TICKS_PER_SECOND = 28 * 6000 / 60 = 2800
            telemetry.addData("Velocity", motor.getVelocity());

            gp.readButtons();

            telemetry.update();
        }

    }

}

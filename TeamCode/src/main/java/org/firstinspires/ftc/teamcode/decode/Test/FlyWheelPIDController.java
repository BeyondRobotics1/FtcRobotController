package org.firstinspires.ftc.teamcode.decode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

@Config
@TeleOp(name = "FlyWheelPIDController Tuner", group = "Decode Test")

public class FlyWheelPIDController extends LinearOpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    PIDController controller;
    public static double kP = 0.001;//0.001; //0.8
    public static double kI = 0.25;//0.25; //0.01
    public static double kD = 0;
    public static double kF = 0.75;

    //public static double targetSpeed = 0.59;
    //public static double targetSpeed = 0.49;
    public static double targetSpeed = 0.4;
    public static double targetVelocity;

    //COUNTS_PER_MOTOR_REV    = 28.0;
    //MOTOR MAX RMP = 6000;
    //ACHIEVABLE_MAX_TICKS_PER_SECOND = 28 * 6000 / 60 = 2800;
    public int kACHIEVABLE_MAX_TICKS_PER_SECOND = 2800; //

    private DcMotorEx leftFlywheel, rightFlywheel;
    @Override
    public void runOpMode() {

        controller = new PIDController(kP, kI, kD);

        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");

        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive())
        {


            if(targetSpeed > 0.5)
                kI = 0.3;

           // if(gamepad1.a) {

                hubs.forEach(LynxModule::clearBulkCache);

                targetVelocity = targetSpeed * kACHIEVABLE_MAX_TICKS_PER_SECOND;
                controller.setPID(kP, kI, kD);

                double velocity = Math.abs(rightFlywheel.getVelocity());

                double pid = controller.calculate(velocity, targetVelocity);

                double ff = kF * targetSpeed;

                double power = pid + ff;

                //setPower(power);
                setPower(power);

                telemetry.addData("Flywheel Velocity (RPM)", "%.1f",  velocity);
                telemetry.addData("Target Velocity (RPM)", "%.1f",  targetVelocity);

                telemetry.addData("PID", "%.5f",  pid);
                telemetry.addData("Feedforward", "%.5f",  ff);
                telemetry.addData("Motor Power", "%.5f",  power);

                telemetry.update();
           // }
        }
    }

    //shoot power is negative
    public void setPower(double power)
    {
        double localPower = -power;//-Math.abs(power);
        leftFlywheel.setPower(localPower);
        rightFlywheel.setPower(localPower);
    }

}

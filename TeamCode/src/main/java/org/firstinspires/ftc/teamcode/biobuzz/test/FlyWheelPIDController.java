package org.firstinspires.ftc.teamcode.biobuzz.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

//@Disabled

@Config
@TeleOp(name = "FlyWheelPIDController Tuner", group = "BioBuzz Test")
public class FlyWheelPIDController  extends LinearOpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    PIDController controller;
    public static double kP = 4;//
    public static double kI = 0.21;//0.15;//0.25; //0.01
    public static double kD = 0;
    public static double kF = 0.9;

    public static double targetSpeed = 0.53;
    public static double targetVelocity;

    //COUNTS_PER_MOTOR_REV    = 28.0;
    //MOTOR MAX RMP = 6000;
    //ACHIEVABLE_MAX_TICKS_PER_SECOND = 28 * 6000 / 60 = 2800;
    public int kACHIEVABLE_MAX_TICKS_PER_SECOND = 2800; //

    private DcMotorEx flywheel = null;
    @Override
    public void runOpMode() {

        telemetry.addData("gamepad1.a", "Pollen FlyWheel");
        telemetry.addData("gamepad1.b", "Nectar FlyWheel");

        telemetry.update();

        controller = new PIDController(kP, kI, kD);

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive())
        {

            if(gamepad1.a) {
                flywheel = hardwareMap.get(DcMotorEx.class, "pollenFlywheel");

                flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

                flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else if(gamepad1.b)
            {
                flywheel = hardwareMap.get(DcMotorEx.class, "nectarFlywheel");

                flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

                flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if(flywheel != null) {
                hubs.forEach(LynxModule::clearBulkCache);

                targetVelocity = targetSpeed * kACHIEVABLE_MAX_TICKS_PER_SECOND;
                controller.setPID(kP, kI, kD);

                double velocity = Math.abs(flywheel.getVelocity());
                double normalizedVelocity = velocity / kACHIEVABLE_MAX_TICKS_PER_SECOND;

                double pid = controller.calculate(normalizedVelocity, targetSpeed);
                double ff = kF * targetSpeed;
                double power = Range.clip(pid + ff, 0.0, 1.0);

                setPower(power);

                telemetry.addData("Flywheel Velocity (ticks/s)", "%.1f", velocity);
                telemetry.addData("Target Velocity (ticks/s)", "%.1f", targetVelocity);

                telemetry.addData("PID", "%.5f", pid);
                telemetry.addData("Feedforward", "%.5f", ff);
                telemetry.addData("Motor Power", "%.5f", power);

                telemetry.update();
            }
        }
    }

    //shoot power is negative
    public void setPower(double power)
    {
        double localPower = -power;//-Math.abs(power);
        flywheel.setPower(localPower);
    }

}

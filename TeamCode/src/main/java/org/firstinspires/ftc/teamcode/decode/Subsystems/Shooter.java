package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    public enum ShootingLocation
    {
        Far,
        Medium,
        Near
    }

    ShootingLocation shooterPosition;

    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private LinearOpMode mode;


    PIDController controller;

    private MotorGroup flyWheel;
    public static double kP = 0.001; //0.8
    public static double kI = 0.2; //0.01
    public static double kD = 0;
    public static double kF = 0.75;


    double targetSpeedFar = 0.6;
    double targetSpeedMedium = 0.485;
    double targetSpeedClose = 0.423;//0.43

    //COUNTS_PER_MOTOR_REV    = 28.0;
    //MOTOR MAX RMP = 6000;
    //ACHIEVABLE_MAX_TICKS_PER_SECOND = 28 * 6000 / 60 = 2800;
    public static int kACHIEVABLE_MAX_TICKS_PER_SECOND = 2800; //

    private boolean isFlyWheelReady;
    public static double targetVelocity;

    public Shooter(HardwareMap hardwareMap, LinearOpMode linearOpMode)
    {
        this.mode = linearOpMode;

        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");

        rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(kP, kI, kD);

        shooterPosition = ShootingLocation.Medium;
        isFlyWheelReady = false;
    }

    //this method should be called in the loop
    //all the time
    public void shoot()
    {
        controller.setPID(kP, kI, kD);

        double targetSpeed = targetSpeedMedium;

        if(shooterPosition == ShootingLocation.Near)
            targetSpeed = targetSpeedClose;
        else if (shooterPosition == ShootingLocation.Far)
            targetSpeed = targetSpeedFar;

        targetVelocity = targetSpeed * kACHIEVABLE_MAX_TICKS_PER_SECOND;

        //motor could give us negative velocity
        double velocity = Math.abs(rightFlywheel.getVelocity());

        double pid = controller.calculate(velocity, targetVelocity);

        double ff = kF * targetSpeed;

        double power = pid + ff;

        setPower(power);

        if(Math.abs(velocity - targetVelocity) <= 100)
            isFlyWheelReady = true;
        else
            isFlyWheelReady = false;

        mode.telemetry.addLine("Shooter Status");
        mode.telemetry.addData("Flywheel Velocity", velocity);
        mode.telemetry.addData("Target Velocity", targetVelocity);

        mode.telemetry.addData("Flywheel PID", pid);
        mode.telemetry.addData("Flywheel FF", ff);
        mode.telemetry.addData("Flywheel Motor Power", power);

        //flyWheel.set(targetSpeedFar);
    }

    public void setShootingLocation(ShootingLocation position)
    {
        shooterPosition = position;
    }

    public boolean isFlyWheelReady()
    {
        return isFlyWheelReady;
    }

    //shoot power is negative
    public void setPower(double power)
    {
        double localPower = - Math.abs(power);
        leftFlywheel.setPower(localPower);
        rightFlywheel.setPower(localPower);
    }

    public void stop()
    {
        leftFlywheel.setPower(0);
        rightFlywheel.setPower(0);
    }
}

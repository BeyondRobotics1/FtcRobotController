package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.Subsystems.Shooter;

public class PollenShooter {

    private DcMotorEx flywheel;
    private LinearOpMode mode;

    PIDController controller;

    //
    public static double kP = 0.01; //0.01
    public static double kI = 0.1; //0.1
    public static double kD = 0;
    public static double kF = 0.9;

    double targetSpeedFarFar = 0.43;//0.43
    double targetSpeedFar = 0.405;//0.41
    double targetSpeedMedium = 0.39;//, 0.395
    double targetSpeedNear = 0.375;//0.395


    private boolean isFlyWheelReady;
    public static double targetVelocity;
    public static double targetSpeed;

    public PollenShooter(HardwareMap hardwareMap, LinearOpMode linearOpMode, int alliance) {
        this.mode = linearOpMode;

        flywheel = hardwareMap.get(DcMotorEx.class, "pollenFlywheel");

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new PIDController(kP, kI, kD);

        targetSpeed = targetSpeedMedium;

        isFlyWheelReady = false;
    }

    public void doFlyWheelVelocityPID()
    {
        controller.setPID(kP, kI, kD);

        targetVelocity = targetSpeed * 2800;

        //motor could give us negative velocity
        double velocity = Math.abs(flywheel.getVelocity());
        if (velocity < 5)
            return;

        double pid = controller.calculate(velocity, targetVelocity);

        double ff = kF * targetSpeed;

        double power = pid + ff;

        //calculated power could be negative
        //We can't use the negative power which cause motor damage
        if (power < 0) {
            //mode.telemetry.addData("Negative PID Power", power);
            power = 0.0005;//0.0
        }

        //mode.telemetry.addData("Power Applied", power);

        setPower(power);

        if(Math.abs(Math.abs(velocity) - targetVelocity) <= 100)
            isFlyWheelReady = true;
        else
            isFlyWheelReady = false;
    }

    //shoot power is negative
    public void setPower(double power)
    {
        double localPower = -power;//
        flywheel.setPower(localPower);
    }

    public void stop()
    {
        flywheel.setPower(0);
    }
}

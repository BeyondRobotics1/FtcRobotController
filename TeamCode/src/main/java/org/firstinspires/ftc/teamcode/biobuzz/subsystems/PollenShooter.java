package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PollenShooter {

    final public int kACHIEVABLE_MAX_TICKS_PER_SECOND = 2800; //

    private DcMotorEx flywheel;
    private LinearOpMode mode;

    private PIDController controller;

    //PID parameters
    private double kP = 0.035; //0.01
    private double kI = 0.1; //0.1
    private double kD = 0;
    private double kF = 0.9;

    private double targetSpeedFarFar = 0.55;//0.43
    private double targetSpeedFar = 0.52;//0.41
    private double targetSpeedMedium = 0.48;//, 0.395
    private double targetSpeedNear = 0.45;//0.395


    private boolean isFlyWheelReady;
    public double targetVelocity;
    public double targetSpeed;

    //used for PID control
    double velocity, pid, ff, power;

    public PollenShooter(HardwareMap hardwareMap,
                         LinearOpMode linearOpMode,
                         int alliance) {
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

        targetVelocity = targetSpeed * kACHIEVABLE_MAX_TICKS_PER_SECOND;

        //motor could give us negative velocity
        velocity = Math.abs(flywheel.getVelocity());
        if (velocity < 5)
            return;

        pid = controller.calculate(velocity, targetVelocity);

        ff = kF * targetSpeed;

        power = pid + ff;

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

    public boolean getIsFryWheelReady()
    {
        return isFlyWheelReady;
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

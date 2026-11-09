package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    final public int kACHIEVABLE_MAX_TICKS_PER_SECOND = 2800;

    public enum ShootingLocation
    {
        AUTO,
        FAR_FAR,
        FAR,
        MEDIUM,
        NEAR
    }

    private DcMotorEx flywheel;
    private LinearOpMode mode;

    private PIDController controller;

    //Parameters for PID
    /*
    Pollen shooter:
        kP = 0.035;
        kI = 0.1;
        kD = 0;
        kF = 0.9;
    Nectar shooter:
        TBD
     */
    private double kP;
    private double kI;
    private double kD;
    private double kF;

    //Target set speed
    /*
    Pollen shooter:
        targetSpeedFarFar = 0.55;
        targetSpeedFar = 0.52;
        targetSpeedMedium = 0.48;
        targetSpeedNear = 0.45;
    Nectar shooter:
        TBD
     */
    private double targetSpeedFarFar;
    private double targetSpeedFar;
    private double targetSpeedMedium;
    private double targetSpeedNear;

    //Internal states
    private boolean isFlyWheelReady;
    private double targetVelocity;
    private double targetSpeed;

    ShootingLocation shootingLocation = ShootingLocation.MEDIUM;
    //used for PID control
    private double velocity, pid, ff, power;

    public Shooter(HardwareMap hardwareMap,
                   LinearOpMode mode,
                   String motorName,
                   double initKP,
                   double initKI,
                   double initKD,
                   double initKF,
                   double initTargetSpeedFarFar,
                   double initTargetSpeedFar,
                   double initTargetSpeedMedium,
                   double initTargetSpeedNear){
        this.mode = mode;
        flywheel = hardwareMap.get(DcMotorEx.class, motorName);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kP = initKP;
        kI = initKI;
        kD = initKD;
        kF = initKF;

        targetSpeedFarFar = initTargetSpeedFarFar;
        targetSpeedFar = initTargetSpeedFar;
        targetSpeedMedium = initTargetSpeedMedium;
        targetSpeedNear = initTargetSpeedNear;

        controller = new PIDController(kP, kI, kD);

        targetSpeed = initTargetSpeedMedium;

        isFlyWheelReady = false;
    }

    public void setPIDParameters(double newKP, double newKI, double newKD, double newKF){
        kP = newKP;
        kI = newKI;
        kD = newKD;
        kF = newKF;
    }

    public boolean getIsFlyWheelReady(){
        return isFlyWheelReady;
    }

    public void doFlyWheelPID(){

        /*
        if (shootingLocation == ShootingLocation.NEAR)
            targetSpeed = targetSpeedNear;
        else if (shootingLocation == ShootingLocation.FAR)
            targetSpeed = targetSpeedFar;
        else if (shootingLocation == ShootingLocation.MEDIUM)
            targetSpeed = targetSpeedMedium;
        else if (shootingLocation == ShootingLocation.FAR_FAR)
            targetSpeed = targetSpeedFarFar;
        else
            targetSpeed = targetSpeedMedium;
        */

        controller.setPID(kP, kI, kD);

        targetVelocity = targetSpeed * kACHIEVABLE_MAX_TICKS_PER_SECOND;

        velocity = Math.abs(flywheel.getVelocity());
        //if (velocity < 5)
        //    return;

        pid = controller.calculate(velocity, targetVelocity);
        ff = kF * targetSpeed;
        power = pid + ff;

        mode.telemetry.addData("PID targetVelocity", targetVelocity);
        mode.telemetry.addData("PID velocity", velocity);
        mode.telemetry.addData("PID pid", pid);
        mode.telemetry.addData("PID ff", ff);
        mode.telemetry.addData("PID power", power);

        //calculated power could be negative
        //We can't use the negative power which cause motor damage
        if (power < 0) {
            //mode.telemetry.addData("Negative PID Power", power);
            power = 0.4;//0.0
        }

        mode.telemetry.addData("Power Applied", power);
        mode.telemetry.update();

        setPower(power);

        if(Math.abs(Math.abs(velocity) - targetVelocity) <= 100)
            isFlyWheelReady = true;
        else
            isFlyWheelReady = false;
    }

    public void setPower(double newPower){
        double localPower = -power;//
        flywheel.setPower(localPower);
    }

    public void stop(){
        flywheel.setPower(0);
    }

    public void setShootingLocation(Shooter.ShootingLocation position)
    {
        shootingLocation = position;
    }

    public ShootingLocation getShooterPosition()
    {
        return shootingLocation;
    }
}

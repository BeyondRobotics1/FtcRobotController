package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

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

    //Parameters for Pollen PID
    public static double kPPollen = 4.5;//0.35;
    public static double kIPollen = 0.21;//0.1;
    public static double kDPollen = 0;
    public static double kFPollen = 0.9;

    //Parameters for Nectar PID
    public static double kPNectar = 0.35;
    public static double kINectar = 0.1;
    public static double kDNectar = 0;
    public static double kFNectar = 0.9;

    //shooting speed for Pollen
    public static double kTargetSpeedFarFarPollen = 0.55;
    public static double kTargetSpeedFarPollen = 0.55;
    public static double kTargetSpeedMediumPollen = 0.55;
    public static double kTargetSpeedNearPollen = 0.55;

    //shooting speed for Nectar
    public static double kTargetSpeedFarFarNectar = 0.6;
    public static double kTargetSpeedFarNectar = 0.6;
    public static double kTargetSpeedMediumNectar = 0.6;
    public static double kTargetSpeedNearNectar = 0.6;


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

        controller.setPID(kP, kI, kD);
    }

    public boolean getIsFlyWheelReady(){
        return isFlyWheelReady;
    }

    public void doFlyWheelPID(){
        switch (shootingLocation) {
            case NEAR:
                targetSpeed = targetSpeedNear;
                break;
            case FAR:
                targetSpeed = targetSpeedFar;
                break;
            case FAR_FAR:
                targetSpeed = targetSpeedFarFar;
                break;
            case AUTO:
            case MEDIUM:
            default:
                targetSpeed = targetSpeedMedium;
                break;
        }

        targetVelocity = targetSpeed * kACHIEVABLE_MAX_TICKS_PER_SECOND;

        velocity = Math.abs(flywheel.getVelocity());

        // Work in normalized motor speed so the gains do not depend on encoder CPR.
        double normalizedVelocity = velocity / kACHIEVABLE_MAX_TICKS_PER_SECOND;
        pid = controller.calculate(normalizedVelocity, targetSpeed);
        ff = kF * targetSpeed;
        power = Range.clip(pid + ff, 0.0, 1.0);

        mode.telemetry.addData("PID targetVelocity", targetVelocity);
        mode.telemetry.addData("PID velocity", velocity);
        mode.telemetry.addData("PID pid", pid);
        mode.telemetry.addData("PID ff", ff);
        mode.telemetry.addData("PID power", power);

        mode.telemetry.addData("Power Applied", power);

        setPower(power);

        isFlyWheelReady = Math.abs(velocity - targetVelocity) <= 100;
    }

    public void setPower(double newPower){
        double localPower = -Range.clip(newPower, 0.0, 1.0);
        flywheel.setPower(localPower);
    }

    public void stop(){
        flywheel.setPower(0);
        isFlyWheelReady = false;
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

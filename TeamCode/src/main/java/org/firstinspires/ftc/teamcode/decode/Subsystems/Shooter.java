package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.OpMode.DecodeBlackBoard;

public class Shooter {

    public enum ShootingLocation
    {
        OUT_ZONE,
        FAR,
        MEDIUM,
        NEAR
    }

    ShootingLocation shooterPosition;

    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private LinearOpMode mode;


    PIDController controller;

    private MotorGroup flyWheel;
    public static double kP = 0.002; //0.001
    public static double kI = 0.25; //0.25
    public static double kD = 0;
    public static double kF = 0.75;


    double targetSpeedOutZone = 0.52; //0.58
    double targetSpeedFar = 0.44; //0.445
    double targetSpeedMedium = 0.42; //0.42
    double targetSpeedNear = 0.394;//0.391

    double blueTargetSpeedOffset = 0.05;

    //COUNTS_PER_MOTOR_REV    = 28.0;
    //MOTOR MAX RMP = 6000;
    //ACHIEVABLE_MAX_TICKS_PER_SECOND = 28 * 6000 / 60 = 2800;
    public static int kACHIEVABLE_MAX_TICKS_PER_SECOND = 2800; //

    private boolean isFlyWheelReady;
    public static double targetVelocity;
    public static double targetSpeed;

    public Shooter(HardwareMap hardwareMap, LinearOpMode linearOpMode, int alliance)
    {
        this.mode = linearOpMode;

        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");

        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new PIDController(kP, kI, kD);

        shooterPosition = ShootingLocation.MEDIUM;
        isFlyWheelReady = false;

        if( alliance == DecodeBlackBoard.BLUE)
        {
            targetSpeedOutZone = 0.53; //0.52
            targetSpeedFar = 0.447; //0.44
            targetSpeedMedium = 0.426; //0.42
            targetSpeedNear = 0.395;//0.391
        }

        targetSpeed = targetSpeedMedium;
    }

    //this method should be called in the loop
    //all the time
    public void shoot() {
        if (shooterPosition == ShootingLocation.NEAR)
            targetSpeed = targetSpeedNear;
        else if (shooterPosition == ShootingLocation.FAR)
            targetSpeed = targetSpeedFar;
        else if (shooterPosition == ShootingLocation.OUT_ZONE)
            targetSpeed = targetSpeedOutZone;
        else
            targetSpeed = targetSpeedMedium;


        //far zone use different kI
        if (targetSpeed > 0.5)
            kI = 0.3;
        else
            kI = 0.25;

        controller.setPID(kP, kI, kD);

        targetVelocity = targetSpeed * kACHIEVABLE_MAX_TICKS_PER_SECOND;

        //motor could give us negative velocity
        double velocity = Math.abs(rightFlywheel.getVelocity());
        if (velocity < 5)
            return;

        double pid = controller.calculate(velocity, targetVelocity);

        double ff = kF * targetSpeed;

        double power = pid + ff;


//        mode.telemetry.addLine("Shooter Status ----------");
//        mode.telemetry.addData("Target Velocity", targetVelocity);
//        mode.telemetry.addData("Flywheel Velocity", velocity);
//        mode.telemetry.addData("Flywheel PID", pid);
//        mode.telemetry.addData("Flywheel FF", ff);
//        mode.telemetry.addData("PID Power", power);

        //calculated power could be negative
        //We can't use the negative power which cause motor damage
        if (power < 0) {
            //mode.telemetry.addData("Negative PID Power", power);
            power = 0.0;
        }

        //mode.telemetry.addData("Power Applied", power);

        setPower(power);

        if(Math.abs(Math.abs(velocity) - targetVelocity) <= 100)
            isFlyWheelReady = true;
        else
            isFlyWheelReady = false;
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
        double localPower = -power;//-Math.abs(power);
        leftFlywheel.setPower(localPower);
        rightFlywheel.setPower(localPower);
    }

    public void stop()
    {
        leftFlywheel.setPower(0);
        rightFlywheel.setPower(0);
    }
}

package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Flywheel controller that uses the REV Hub's RUN_USING_ENCODER velocity PIDF loop.
 *
 * <p>Target speeds are expressed as a fraction of {@code maxTicksPerSecond}. The Hub runs
 * the time-critical control loop; {@link #doFlyWheelPID()} selects the requested speed and
 * updates the ready-to-fire state.</p>
 *
 * <p>The PIDF values accepted by this class are REV Hub velocity PIDF coefficients. They are
 * not interchangeable with the normalized FTCLib PID values used by {@link Shooter}.</p>
 */
public class VelocityPIDFShooter {
    public static final double DEFAULT_MAX_TICKS_PER_SECOND = 2800.0;
    public static final double DEFAULT_READY_TOLERANCE_TICKS_PER_SECOND = 100.0;
    public static final double DEFAULT_READY_STABLE_TIME_SECONDS = 0.15;

    public enum ShootingLocation {
        AUTO,
        FAR_FAR,
        FAR,
        MEDIUM,
        NEAR
    }

    private final DcMotorEx flywheel;
    private final LinearOpMode mode;
    private final double maxTicksPerSecond;

    private final double targetSpeedFarFar;
    private final double targetSpeedFar;
    private final double targetSpeedMedium;
    private final double targetSpeedNear;

    private ShootingLocation shootingLocation = ShootingLocation.MEDIUM;
    private double targetSpeed;
    private double targetVelocity;
    private double readyToleranceTicksPerSecond = DEFAULT_READY_TOLERANCE_TICKS_PER_SECOND;
    private long readyStableTimeNanos = secondsToNanos(DEFAULT_READY_STABLE_TIME_SECONDS);
    private long withinToleranceSinceNanos = -1;
    private boolean isFlyWheelReady;

    /**
     * Uses the velocity PIDF coefficients supplied by the configured motor type.
     */
    public VelocityPIDFShooter(HardwareMap hardwareMap,
                               LinearOpMode mode,
                               String motorName,
                               double initTargetSpeedFarFar,
                               double initTargetSpeedFar,
                               double initTargetSpeedMedium,
                               double initTargetSpeedNear) {
        this(hardwareMap,
                mode,
                motorName,
                DEFAULT_MAX_TICKS_PER_SECOND,
                initTargetSpeedFarFar,
                initTargetSpeedFar,
                initTargetSpeedMedium,
                initTargetSpeedNear);
    }

    /**
     * Uses the velocity PIDF coefficients supplied by the configured motor type.
     */
    public VelocityPIDFShooter(HardwareMap hardwareMap,
                               LinearOpMode mode,
                               String motorName,
                               double maxTicksPerSecond,
                               double initTargetSpeedFarFar,
                               double initTargetSpeedFar,
                               double initTargetSpeedMedium,
                               double initTargetSpeedNear) {
        if (maxTicksPerSecond <= 0) {
            throw new IllegalArgumentException("maxTicksPerSecond must be greater than zero");
        }

        this.mode = mode;
        this.maxTicksPerSecond = maxTicksPerSecond;
        targetSpeedFarFar = clipTargetSpeed(initTargetSpeedFarFar);
        targetSpeedFar = clipTargetSpeed(initTargetSpeedFar);
        targetSpeedMedium = clipTargetSpeed(initTargetSpeedMedium);
        targetSpeedNear = clipTargetSpeed(initTargetSpeedNear);
        targetSpeed = targetSpeedMedium;

        flywheel = hardwareMap.get(DcMotorEx.class, motorName);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Convenience constructor that applies explicit REV Hub velocity PIDF coefficients.
     */
    public VelocityPIDFShooter(HardwareMap hardwareMap,
                               LinearOpMode mode,
                               String motorName,
                               double initKP,
                               double initKI,
                               double initKD,
                               double initKF,
                               double initTargetSpeedFarFar,
                               double initTargetSpeedFar,
                               double initTargetSpeedMedium,
                               double initTargetSpeedNear) {
        this(hardwareMap,
                mode,
                motorName,
                initTargetSpeedFarFar,
                initTargetSpeedFar,
                initTargetSpeedMedium,
                initTargetSpeedNear);
        setPIDParameters(initKP, initKI, initKD, initKF);
    }

    /** Applies PIDF coefficients to the Hub's RUN_USING_ENCODER velocity controller. */
    public void setPIDParameters(double newKP, double newKI, double newKD, double newKF) {
        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(newKP, newKI, newKD, newKF));
    }

    public PIDFCoefficients getPIDParameters() {
        return flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Selects the location target, commands it with setVelocity(), and updates readiness.
     * Call once per active OpMode loop.
     */
    public void doFlyWheelPID() {
        setTargetSpeed(getSelectedTargetSpeed());
        updateReadiness();

        if (mode != null) {
            mode.telemetry.addData("Flywheel target (ticks/s)", targetVelocity);
            mode.telemetry.addData("Flywheel velocity (ticks/s)", getCurrentVelocity());
            mode.telemetry.addData("Flywheel error (ticks/s)", getVelocityError());
            mode.telemetry.addData("Flywheel ready", isFlyWheelReady);
        }
    }

    /**
     * Commands a normalized velocity. This method replaces the old class's open-loop power
     * behavior so existing warm-up call sites can remain velocity controlled.
     */
    public void setPower(double normalizedSpeed) {
        setTargetSpeed(normalizedSpeed);
    }

    public void setTargetSpeed(double normalizedSpeed) {
        double newTargetSpeed = clipTargetSpeed(normalizedSpeed);
        double newTargetVelocity = newTargetSpeed * maxTicksPerSecond;

        if (Math.abs(newTargetVelocity - targetVelocity) > 1e-6) {
            resetReadiness();
        }

        targetSpeed = newTargetSpeed;
        targetVelocity = newTargetVelocity;
        flywheel.setVelocity(targetVelocity);
    }

    public void setTargetVelocity(double ticksPerSecond) {
        double clippedVelocity = Range.clip(ticksPerSecond, 0.0, maxTicksPerSecond);
        setTargetSpeed(clippedVelocity / maxTicksPerSecond);
    }

    public void stop() {
        targetSpeed = 0;
        targetVelocity = 0;
        flywheel.setVelocity(0);
        resetReadiness();
    }

    public boolean getIsFlyWheelReady() {
        return isFlyWheelReady;
    }

    public double getCurrentVelocity() {
        return Math.abs(flywheel.getVelocity());
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getVelocityError() {
        return targetVelocity - getCurrentVelocity();
    }

    public double getCurrentAmps() {
        return flywheel.getCurrent(CurrentUnit.AMPS);
    }

    public double getAppliedPower() {
        return Math.abs(flywheel.getPower());
    }

    public void setShootingLocation(ShootingLocation position) {
        if (position == null) {
            throw new IllegalArgumentException("position cannot be null");
        }

        if (shootingLocation != position) {
            shootingLocation = position;
            resetReadiness();
        }
    }

    public ShootingLocation getShooterPosition() {
        return shootingLocation;
    }

    /**
     * Sets how close and for how long the measured velocity must remain near the target.
     */
    public void setReadyCriteria(double toleranceTicksPerSecond, double stableTimeSeconds) {
        if (toleranceTicksPerSecond < 0 || stableTimeSeconds < 0) {
            throw new IllegalArgumentException("Ready tolerance and stable time cannot be negative");
        }

        readyToleranceTicksPerSecond = toleranceTicksPerSecond;
        readyStableTimeNanos = secondsToNanos(stableTimeSeconds);
        resetReadiness();
    }

    private double getSelectedTargetSpeed() {
        switch (shootingLocation) {
            case NEAR:
                return targetSpeedNear;
            case FAR:
                return targetSpeedFar;
            case FAR_FAR:
                return targetSpeedFarFar;
            case AUTO:
            case MEDIUM:
            default:
                return targetSpeedMedium;
        }
    }

    private void updateReadiness() {
        if (targetVelocity <= 0
                || Math.abs(getVelocityError()) > readyToleranceTicksPerSecond) {
            resetReadiness();
            return;
        }

        long now = System.nanoTime();
        if (withinToleranceSinceNanos < 0) {
            withinToleranceSinceNanos = now;
        }

        isFlyWheelReady = now - withinToleranceSinceNanos >= readyStableTimeNanos;
    }

    private void resetReadiness() {
        withinToleranceSinceNanos = -1;
        isFlyWheelReady = false;
    }

    private static double clipTargetSpeed(double normalizedSpeed) {
        return Range.clip(normalizedSpeed, 0.0, 1.0);
    }

    private static long secondsToNanos(double seconds) {
        return (long) (seconds * 1_000_000_000L);
    }
}

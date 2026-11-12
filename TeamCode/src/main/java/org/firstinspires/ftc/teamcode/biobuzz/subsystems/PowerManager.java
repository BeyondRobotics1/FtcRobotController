package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Protects the robot from high-current voltage sag by reducing drivetrain load.
 * Shooter power is left under PID control so shot velocity remains predictable.
 */
public final class PowerManager {
    public enum PowerState {
        NORMAL,
        LIMITED,
        CRITICAL
    }

    public static final double LIMITED_VOLTAGE = 10.5; //11.5
    public static final double CRITICAL_VOLTAGE = 9.5;//10.8
    public static final double VOLTAGE_RECOVERY_MARGIN = 0.3;

    public static final double LIMITED_TOTAL_CURRENT_AMPS = 14.0;//14
    public static final double CRITICAL_TOTAL_CURRENT_AMPS = 18;//18
    public static final double CURRENT_RECOVERY_MARGIN_AMPS = 2.0;

    public static final double NORMAL_DRIVE_SCALE = 1.0; //1.0
    public static final double LIMITED_DRIVE_SCALE = 0.80; //0.7
    public static final double CRITICAL_DRIVE_SCALE = 0.6; //0.45

    private static final double FILTER_ALPHA = 0.25;

    private final HardwareMap hardwareMap;

    private PowerState state = PowerState.NORMAL;
    private double batteryVoltage = Double.NaN;
    private double driveCurrentAmps = Double.NaN;
    private double shooterCurrentAmps = Double.NaN;
    private double totalCurrentAmps;

    public PowerManager(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void update(double measuredDriveCurrentAmps, double measuredShooterCurrentAmps) {
        double measuredVoltage = readBatteryVoltage();
        batteryVoltage = filterVoltage(batteryVoltage, measuredVoltage);
        driveCurrentAmps = filterCurrent(driveCurrentAmps, measuredDriveCurrentAmps);
        shooterCurrentAmps = filterCurrent(shooterCurrentAmps, measuredShooterCurrentAmps);
        totalCurrentAmps = driveCurrentAmps + shooterCurrentAmps;

        boolean criticalLoad = batteryVoltage <= CRITICAL_VOLTAGE
                || totalCurrentAmps >= CRITICAL_TOTAL_CURRENT_AMPS;
        boolean limitedLoad = batteryVoltage <= LIMITED_VOLTAGE
                || totalCurrentAmps >= LIMITED_TOTAL_CURRENT_AMPS;

        switch (state) {
            case NORMAL:
                if (criticalLoad) {
                    state = PowerState.CRITICAL;
                } else if (limitedLoad) {
                    state = PowerState.LIMITED;
                }
                break;

            case LIMITED:
                if (criticalLoad) {
                    state = PowerState.CRITICAL;
                } else if (batteryVoltage >= LIMITED_VOLTAGE + VOLTAGE_RECOVERY_MARGIN
                        && totalCurrentAmps
                        <= LIMITED_TOTAL_CURRENT_AMPS - CURRENT_RECOVERY_MARGIN_AMPS) {
                    state = PowerState.NORMAL;
                }
                break;

            case CRITICAL:
                if (batteryVoltage >= CRITICAL_VOLTAGE + VOLTAGE_RECOVERY_MARGIN
                        && totalCurrentAmps
                        <= CRITICAL_TOTAL_CURRENT_AMPS - CURRENT_RECOVERY_MARGIN_AMPS) {
                    state = PowerState.LIMITED;
                }
                break;
        }
    }

    public double getDrivePowerScale() {
        switch (state) {
            case CRITICAL:
                return CRITICAL_DRIVE_SCALE;
            case LIMITED:
                return LIMITED_DRIVE_SCALE;
            case NORMAL:
            default:
                return NORMAL_DRIVE_SCALE;
        }
    }

    public PowerState getState() {
        return state;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Power state", state);
        telemetry.addData("Battery voltage", "%.2f V", batteryVoltage);
        telemetry.addData("Drive current", "%.1f A", driveCurrentAmps);
        telemetry.addData("Shooter current", "%.1f A", shooterCurrentAmps);
        telemetry.addData("Managed current", "%.1f A", totalCurrentAmps);
        telemetry.addData("Drive power scale", "%.0f%%", getDrivePowerScale() * 100.0);
    }

    private double readBatteryVoltage() {
        double lowestVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0.0 && voltage < lowestVoltage) {
                lowestVoltage = voltage;
            }
        }

        // No voltage sensor should be unusual. Treat it as healthy so current
        // protection remains available without permanently limiting the drive.
        return Double.isFinite(lowestVoltage) ? lowestVoltage : 12.0;
    }

    private static double filterVoltage(double previous, double measurement) {
        if (!Double.isFinite(previous)) {
            return measurement;
        }

        // React to a sag immediately, but require a sustained recovery before
        // restoring drivetrain power.
        if (measurement < previous) {
            return measurement;
        }
        return previous + FILTER_ALPHA * (measurement - previous);
    }

    private static double filterCurrent(double previous, double measurement) {
        if (!Double.isFinite(previous)) {
            return measurement;
        }

        // React to a current spike immediately, then decay the reported load
        // gradually so the limiter does not chatter between states.
        if (measurement > previous) {
            return measurement;
        }
        return previous + FILTER_ALPHA * (measurement - previous);
    }
}

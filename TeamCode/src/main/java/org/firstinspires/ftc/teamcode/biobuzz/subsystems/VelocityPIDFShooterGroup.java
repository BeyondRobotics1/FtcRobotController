package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Groups the BioBuzz flywheels that use the REV Hub velocity PIDF controller. */
public class VelocityPIDFShooterGroup {
    private final VelocityPIDFShooter pollenShooter;
    private final VelocityPIDFShooter nectarShooter;

    public VelocityPIDFShooterGroup(HardwareMap hardwareMap, LinearOpMode linearOpMode) {
        pollenShooter = new VelocityPIDFShooter(
                hardwareMap,
                linearOpMode,
                "pollenFlywheel",
                Shooter.kTargetSpeedFarFarPollen,
                Shooter.kTargetSpeedFarPollen,
                Shooter.kTargetSpeedMediumPollen,
                Shooter.kTargetSpeedNearPollen);

        // Enable when the nectar flywheel is installed and configured.
//        nectarShooter = new VelocityPIDFShooter(
//                hardwareMap,
//                linearOpMode,
//                "nectarFlywheel",
//                Shooter.kTargetSpeedFarFarNectar,
//                Shooter.kTargetSpeedFarNectar,
//                Shooter.kTargetSpeedMediumNectar,
//                Shooter.kTargetSpeedNearNectar);
        nectarShooter = null;
    }

    public void doFlyWheelPID() {
        pollenShooter.doFlyWheelPID();
        if (nectarShooter != null) {
            nectarShooter.doFlyWheelPID();
        }
    }

    public void stop() {
        pollenShooter.stop();
        if (nectarShooter != null) {
            nectarShooter.stop();
        }
    }

    /** Sets a normalized velocity request for each installed flywheel. */
    public void setPower(double normalizedSpeed) {
        pollenShooter.setPower(normalizedSpeed);
        if (nectarShooter != null) {
            nectarShooter.setPower(normalizedSpeed);
        }
    }

    public void setShootingLocation(VelocityPIDFShooter.ShootingLocation position) {
        pollenShooter.setShootingLocation(position);
        if (nectarShooter != null) {
            nectarShooter.setShootingLocation(position);
        }
    }

    public double getTotalCurrentAmps() {
        double totalCurrent = pollenShooter.getCurrentAmps();
        if (nectarShooter != null) {
            totalCurrent += nectarShooter.getCurrentAmps();
        }
        return totalCurrent;
    }

    public boolean isPollenFlyWheelReady() {
        return pollenShooter.getIsFlyWheelReady();
    }

    public boolean isNectarFlyWheelReady() {
        return nectarShooter != null && nectarShooter.getIsFlyWheelReady();
    }
}

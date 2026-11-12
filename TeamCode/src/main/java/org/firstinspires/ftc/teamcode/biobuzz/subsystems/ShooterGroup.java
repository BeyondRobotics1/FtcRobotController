package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterGroup {
    private Shooter pollenShooter;
    private Shooter nectarShooter;

    public ShooterGroup(HardwareMap hardwareMap, LinearOpMode linearOpMode){
        pollenShooter = new Shooter(hardwareMap,
                linearOpMode,
                "pollenFlywheel",
                Shooter.kPPollen,
                Shooter.kIPollen,
                Shooter.kDPollen,
                Shooter.kFPollen,
                Shooter.kTargetSpeedFarFarPollen,
                Shooter.kTargetSpeedFarPollen,
                Shooter.kTargetSpeedMediumPollen,
                Shooter.kTargetSpeedNearPollen);

//        nectarShooter = new Shooter(hardwareMap,
//                linearOpMode,
//                "nectarFlywheel",
//                Shooter.kPNectar,
//                Shooter.kINectar,
//                Shooter.kDNectar,
//                Shooter.kFNectar,
//                Shooter.kTargetSpeedFarFarNectar,
//                Shooter.kTargetSpeedFarNectar,
//                Shooter.kTargetSpeedMediumNectar,
//                Shooter.kTargetSpeedNearNectar);
    }

    public void doFlyWheelPID(){
        pollenShooter.doFlyWheelPID();
        //nectarShooter.doFlyWheelPID();
    }

    public void stop(){
        pollenShooter.stop();
        //nectarShooter.stop();
    }

    public void setPower(double power)
    {
        pollenShooter.setPower(power);
        //nectarShooter.setPower(power);
    }

    public void setShootingLocation(Shooter.ShootingLocation position)
    {
        pollenShooter.setShootingLocation(position);
       // nectarShooter.setShootingLocation(position);
    }

    public double getTotalCurrentAmps() {
        double totalCurrent = pollenShooter.getCurrentAmps();
        if (nectarShooter != null) {
            totalCurrent += nectarShooter.getCurrentAmps();
        }
        return totalCurrent;
    }

    public boolean isPollenFlyWheelReady(){
        return pollenShooter.getIsFlyWheelReady();
    }
    public boolean isNectarFlyWheelReady(){
        return nectarShooter.getIsFlyWheelReady();
    }
}

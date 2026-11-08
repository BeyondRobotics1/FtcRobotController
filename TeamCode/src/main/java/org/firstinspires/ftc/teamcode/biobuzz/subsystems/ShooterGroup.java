package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterGroup {
    private Shooter pollenShooter;
    private Shooter nectarShooter;

    public ShooterGroup(HardwareMap hardwareMap, LinearOpMode linearOpMode){
        pollenShooter = new Shooter(hardwareMap,
                linearOpMode,
                "pollenShooter",
                0.035,
                0.1,
                0,
                0.9,
                0.55,
                0.52,
                0.48,
                0.45);
        nectarShooter = new Shooter(hardwareMap,
                linearOpMode,
                "nectarShooter",
                0.035,
                0.1,
                0,
                0.9,
                0.55,
                0.52,
                0.48,
                0.45);
    }

    public void doFlyWheelPID(){
        pollenShooter.doFlyWheelPID();
        nectarShooter.doFlyWheelPID();
    }

    public void stop(){
        pollenShooter.stop();
        nectarShooter.stop();
    }

    public void setPower(double power)
    {
        pollenShooter.setPower(power);
        nectarShooter.setPower(power);
    }

    public void setShootingLocation(Shooter.ShootingLocation position)
    {
        pollenShooter.setShootingLocation(position);
        nectarShooter.setShootingLocation(position);
    }

    public boolean isPollenFlyWheelReady(){
        return pollenShooter.getIsFlyWheelReady();
    }
    public boolean isNectarFlyWheelReady(){
        return nectarShooter.getIsFlyWheelReady();
    }
}

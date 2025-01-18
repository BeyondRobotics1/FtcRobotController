package org.firstinspires.ftc.teamcode.centerstage.picasso;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * DroneLauncher class to control the Launcher's trigger
 */
public class DroneLauncher {
    LinearOpMode mode;

    //servo to release the trigger
    Servo launcher;

    public DroneLauncher(HardwareMap hardwareMap, LinearOpMode mode) {
        this.mode = mode;
        launcher = hardwareMap.get(Servo.class, "launcher");
    }

    //release the trigger
    public void ReleaseTrigger()
    {
        launcher.setPosition(0.5);
    }
}

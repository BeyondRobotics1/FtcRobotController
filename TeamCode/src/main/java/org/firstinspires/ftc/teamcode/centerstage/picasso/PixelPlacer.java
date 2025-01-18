package org.firstinspires.ftc.teamcode.centerstage.picasso;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PixelPlacer {

    LinearOpMode mode;
    Servo placer;

    boolean isLocked;

    public PixelPlacer(HardwareMap hardwareMap, LinearOpMode mode)
    {
        this.mode = mode;

        placer = hardwareMap.get(Servo.class, "pixelplacer");
        isLocked = false;

        Lock();
    }

    //Servo arm down to lock in the pixel placed by driver team
    public void Lock()
    {
        if(!isLocked) {
            placer.setPosition(1);
            isLocked = true;
        }
    }

    //Servo arm up to release the pixel on the place detected
    //with team's prop
    public void Unlock()
    {
        if(isLocked) {
            placer.setPosition(0.6);
            isLocked = false;
        }
    }
}

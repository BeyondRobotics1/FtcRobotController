package org.firstinspires.ftc.teamcode.centerstage.picasso;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PixelPlacer {

    LinearOpMode mode;
    Servo placer;

    public PixelPlacer(HardwareMap hardwareMap, LinearOpMode mode)
    {
        this.mode = mode;

        placer = hardwareMap.get(Servo.class, "pixelplacer");
    }

    //Servo arm down to lock in the pixel placed by driver team
    public void GrabPixel()
    {
        placer.setPosition(0);
    }

    //Servo arm up to release the pixel on the place detected
    //with team's prop
    public void PlacePixel()
    {
        placer.setPosition(0.2);
    }
}

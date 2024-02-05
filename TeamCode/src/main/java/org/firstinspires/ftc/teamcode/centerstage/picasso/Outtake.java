package org.firstinspires.ftc.teamcode.centerstage.picasso;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {

    LinearOpMode mode;
    Servo outTake;

    public Outtake(HardwareMap hardwareMap, LinearOpMode mode)
    {
        this.mode = mode;

        outTake = hardwareMap.get(Servo.class, "outtake");
    }

    //take in the pixels, work along with intake
    public void TakeIn()
    {
        outTake.setPosition(0);
    }

    //release pixels on the backdrop
    public void TakeOut()
    {
        //outTake.setPosition(0.8);
        TakeOut(0.8);
    }

    //release pixels on the backdrop
    public void TakeOut(double speed)
    {
        outTake.setPosition(speed);
    }

    //stop moving
    public void Hold()
    {
        outTake.setPosition(0.5);
    }
}


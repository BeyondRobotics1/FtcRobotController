package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * control code for aligner
 * aligner allows our drivers to see if cones are scorable more easily
 * this code sets the servo locations for up and down positions
 */

public class Aligner {
    Servo bar;
    double upPosition = 1;//0.95
    double downPosition = 0.27;
    LinearOpMode mode;

    int currentPosition = 1; //1 - up, 2 - down

    //constructor
    public Aligner(HardwareMap hardwareMap, LinearOpMode mode){
        bar = hardwareMap.get(Servo.class, "bar");
        this.mode = mode;

        //move the aligner to the up position
        bar.setPosition(upPosition);
        currentPosition = 1;
    }

    //move the aligner to the up position
    public void moveUp(){

        if(currentPosition != 1) {
            bar.setPosition(upPosition);
            currentPosition = 1;
        }
    }

    //move the aligner to the down position
    public void moveDown(){

        if(currentPosition != 2) {
            bar.setPosition(downPosition);
            currentPosition = 2;
        }
    }

    //return true is aligner is set to up position
    public boolean isDownPosition()
    {
        return currentPosition == 2;
    }

    //return true is aligner is set to down position
    public boolean isUpPosition()
    {
        return currentPosition == 1;
    }
}

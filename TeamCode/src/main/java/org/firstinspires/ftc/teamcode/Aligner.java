package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * control code for aligner
 * aligner allows our drivers to see if cones are scorable more easily
 * this code sets the servo locations for up and down
 */

public class Aligner {
    Servo bar;
    double upPosition = 1;
    double downPosition = 0.25;
    LinearOpMode mode;

    public Aligner(HardwareMap hardwareMap, LinearOpMode mode){
        bar = hardwareMap.get(Servo.class, "bar");
        this.mode = mode;

    }
    public void moveUp(){
        bar.setPosition(upPosition);
    }
    public void moveDown(){
        bar.setPosition(downPosition);
    }
}

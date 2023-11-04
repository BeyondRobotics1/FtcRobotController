package org.firstinspires.ftc.teamcode.centerstage.mosaico;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MosaicoClaw {

    //servo to close or open the claw
    //close position 0.01,
    //open position 0.095
    Servo claw;

    //servo to rotate the claw
    //down position: 0.91
    //up position: 0.65
    Servo rotor;

    LinearOpMode mode;

    boolean closed = false;
    boolean up = false;

    public MosaicoClaw (HardwareMap hardwareMap, LinearOpMode mode) {

        this.mode = mode;
        claw = hardwareMap.get(Servo.class, "claw");
        rotor = hardwareMap.get(Servo.class, "rotor");
    }

   public void close(){
        claw.setPosition(0.01);
    }

    public void open(){
        claw.setPosition(0.095);
    }

    public void rotate_up(){
        rotor.setPosition(0.69);
    }

    public void rotate_down(){
        rotor.setPosition(0.91);
    }

    public void rotate_middle(){
        rotor.setPosition(0.75);
    }


    {
        //TBD
    }
}

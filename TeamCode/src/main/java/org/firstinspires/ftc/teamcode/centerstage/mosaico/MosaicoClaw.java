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
        //TBD
    }

    public void open(){
        //TBD
    }

    public void rotate_up()
    {
        //TBD
    }

    public void rotate_down()
    {
        //TBD
    }
}

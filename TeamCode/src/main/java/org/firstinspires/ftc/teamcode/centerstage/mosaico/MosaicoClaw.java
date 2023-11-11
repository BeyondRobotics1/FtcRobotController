package org.firstinspires.ftc.teamcode.centerstage.mosaico;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MosaicoClaw {

    double armPosition[] = {1, 0.7, 0.4, 0};
    int currentArmPosition = 0;

    //servo to close or open the claw
    //close position 0.01,
    //open position 0.095
    Servo claw;

    //servo to rotate the claw
    //down position: 0.8
    //up position: 0.55
    Servo arm;

    LinearOpMode mode;

    boolean closed = false;

    public MosaicoClaw (HardwareMap hardwareMap, LinearOpMode mode) {

        this.mode = mode;
        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "rotor");
        arm.scaleRange(0.55, 0.75);

        closed = false;
        open();

        setArmPosition(1);
    }

   public void close(){

        if(!closed) {
            claw.setPosition(0.4);//0.2
            closed = true;
        }
    }

    public void open(){
        if(closed) {
            claw.setPosition(0.5);//
            closed = false;
        }
    }

    public void rotate_up(){
        setArmPosition(3);
    }

    public void rotate_down(){
        setArmPosition(0);
    }

    public void rotate_middle(){
        setArmPosition(2);
    }

    //positionID 0 - down, 1, 2, middle, 3 - up
    public void setArmPosition(double position)
    {
        arm.setPosition(position);
    }
}

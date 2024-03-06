package org.firstinspires.ftc.teamcode.centerstage.picasso;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    LinearOpMode mode;
    Servo arm2;

    boolean isDown;

    public Arm(HardwareMap hardwareMap, LinearOpMode mode)
    {
        this.mode = mode;

        arm2 = hardwareMap.get(Servo.class, "arm2");

        arm2.setDirection(Servo.Direction.REVERSE);

        isDown = false;

        goDown();
    }

    public boolean isArmDown()
    {
        return isDown;
    }

    public void goDown()
    {
        if(!isDown) {


            mode.sleep(2);

            arm2.setPosition(0.32);//0.32

            isDown = true;
        }
    }

    public void goUp()
    {
        if(isDown) {

            arm2.setPosition(0.65);

//            //three step to simulate slower speed
//            //arm1.setPosition(0.3);
//           // arm2.setPosition(0.3);
//            mode.sleep(2);
//
//           // arm1.setPosition(0.6);
//            arm2.setPosition(0.6);
//            mode.sleep(1);
//
//            //arm1.setPosition(0.92);
//            arm2.setPosition(0.92);

            isDown = false;
        }
    }


}

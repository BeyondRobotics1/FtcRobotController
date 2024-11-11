package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Arm {

    LinearOpMode mode;
    DcMotorEx arm;

    boolean isDown;

    public Arm(HardwareMap hardwareMap, LinearOpMode mode)
    {
        this.mode = mode;

        arm = hardwareMap.get(DcMotorEx.class, "arm");

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

            //two step to simulate slower speed
            //arm1.setPosition(0.25);//0.25
            //arm2.setPosition(0.25);

            mode.sleep(2);


            isDown = true;
        }
    }

    public void goUp()
    {
        if(isDown) {

            //three step to simulate slower speed
            //arm1.setPosition(0.3);
            // arm2.setPosition(0.3);
            mode.sleep(2);

        }
    }


}

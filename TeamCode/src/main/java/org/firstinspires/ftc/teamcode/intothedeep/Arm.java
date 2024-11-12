package org.firstinspires.ftc.teamcode.intothedeep;

import android.text.style.UpdateAppearance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Arm {

    LinearOpMode mode;
    DcMotorEx arm1;
    DcMotorEx arm2;
    enum powerMode{
        up,
        down,
        stay
    }
    public Arm(HardwareMap hardwareMap, LinearOpMode mode)
    {
        this.mode = mode;

        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
        arm2 = hardwareMap.get(DcMotorEx.class, "arm2");
    }


}

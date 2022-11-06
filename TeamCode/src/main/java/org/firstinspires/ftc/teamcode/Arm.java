package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    int arm_position_id = 0;
    double arm_positions[] = {1, 0.5, 0};

    Servo servoArm;
    Servo servoClaw;
    DcMotorSimple slideMotor;

    public Arm(HardwareMap hardwareMap)
    {
        //servos
        servoArm = hardwareMap.get(Servo.class,"arm");
        servoClaw = hardwareMap.get(Servo.class,"claw");

        slideMotor = hardwareMap.get(DcMotorSimple.class, "slide");

        servoClaw.scaleRange(0.1, 0.25);
        servoClaw.setPosition(0);

        servoArm.scaleRange(0.15, 0.85);
        servoArm.setPosition(1);
    }

    public void openClaw()
    {
        servoClaw.setPosition(0);
    }

    public void closeClaw()
    {
        servoClaw.setPosition(1.0);
    }

    public void setClawPosition(int arm_position_id)
    {
        this.arm_position_id = arm_position_id;

        servoArm.setPosition(arm_positions[this.arm_position_id]);
    }

    public void moveSlide(double power)
    {
        slideMotor.setPower(Helper.cubicWithSign(power));
    }
}

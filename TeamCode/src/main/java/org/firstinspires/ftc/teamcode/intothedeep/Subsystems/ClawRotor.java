package org.firstinspires.ftc.teamcode.intothedeep.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class    ClawRotor {

    Servo servo;
    LinearOpMode mode;

    //claw is down to pickup sample from the intake
    public final double CLAW_UP_POSITION = 0.02;

    //clas is up to pick up specimen
    public final double CLAW_DOWN_POSITION = 0.7;

    public ClawRotor(HardwareMap hardwareMap, LinearOpMode mode)
    {
        this.mode = mode;

        servo = hardwareMap.get(Servo.class, "outtakeRotateClaw");;;
    }

    public void SetClawUp() {
        servo.setPosition(CLAW_UP_POSITION);
    }

    public void SetClawDown()
    {
        servo.setPosition(CLAW_DOWN_POSITION);
    }

    public void SetPosition(double position)
    {
        servo.setPosition(position);
    }
}

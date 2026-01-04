package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Clutch {

    //our robot has two clutches, left and right
    //controlled by left and right servos accordingly
    private Servo leftClutchServo;
    private Servo rightClutchServo;

    //constructor
    public Clutch(HardwareMap hardwareMap)
    {

        leftClutchServo = hardwareMap.get(Servo.class, "clutchLeft");
        rightClutchServo = hardwareMap.get(Servo.class, "clutchRight");

        leftClutchServo.setDirection(Servo.Direction.REVERSE);
    }

    //engage left and right clutches
    public void engage()
    {
        leftClutchServo.setPosition(0.1);
        rightClutchServo.setPosition(0.09);
    }

    //Disengage left and right clutches
    public void disengage()
    {
        leftClutchServo.setPosition(0);
        rightClutchServo.setPosition(0);
    }

    public double getLeftClutchServoPosition()
    {
        return leftClutchServo.getPosition();
    }

    public double getRightClutchServoPosition()
    {
        return rightClutchServo.getPosition();
    }
}

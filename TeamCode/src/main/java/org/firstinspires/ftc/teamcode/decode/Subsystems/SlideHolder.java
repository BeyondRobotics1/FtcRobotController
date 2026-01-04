package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SlideHolder {

    private Servo leftSlidehServo;
    private Servo rightSlideServo;

    public SlideHolder(HardwareMap hardwareMap)
    {

        leftSlidehServo = hardwareMap.get(Servo.class, "slideLeft");
        rightSlideServo = hardwareMap.get(Servo.class, "slideRight");

        leftSlidehServo.setDirection(Servo.Direction.REVERSE);
    }

    //engage left and right clutches
    public void hold()
    {
        leftSlidehServo.setPosition(0.0);
        rightSlideServo.setPosition(0.0);
    }

    //Disengage left and right clutches
    public void release()
    {
        leftSlidehServo.setPosition(0.5);
        rightSlideServo.setPosition(0.5);
    }

    public double getLeftSlideServoPosition()
    {
        return leftSlidehServo.getPosition();
    }

    public double getRightSlideServoPosition()
    {
        return rightSlideServo.getPosition();
    }
}

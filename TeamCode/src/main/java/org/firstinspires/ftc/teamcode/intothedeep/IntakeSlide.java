package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSlide {
    private Servo servoRight;
    private Servo servoLeft;

    public IntakeSlide(HardwareMap hardwareMap){
        servoLeft = hardwareMap.get(Servo.class, "intakeSlideLeft");
        servoRight = hardwareMap.get(Servo.class, "intakeSlideRight");

        servoLeft.setDirection(Servo.Direction.REVERSE);
    }

    public void MoveIn(){
        servoLeft.setPosition(0);
        servoRight.setPosition(0);
    }
    public void MoveOut(){
        servoLeft.setPosition(1);
        servoRight.setPosition(1);
    }
    public void Hold(){
        servoLeft.setPosition(0.48);
        servoRight.setPosition(0.48);
    }

}

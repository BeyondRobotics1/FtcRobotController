package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSlide {
    private Servo servoRight;
    private Servo servoLeft;
    private DigitalChannel touchSensorLowLimit;

    public IntakeSlide(HardwareMap hardwareMap){
        servoLeft = hardwareMap.get(Servo.class, "intakeSlideLeft");
        servoRight = hardwareMap.get(Servo.class, "intakeSlideRight");

        touchSensorLowLimit =  hardwareMap.get(DigitalChannel.class, "hslideLow");
        touchSensorLowLimit.setMode(DigitalChannel.Mode.INPUT);


        servoLeft.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Move the horizontal slide in and out
     * @param speed (0.5 1] move out, [0 0.5) move in, 0.5 hold
     */
    public void Move(double speed)
    {
        //(0.5 1] move out (extending)
        //[0 0.5) move in (retracting)

        //if retracting and touch sensor is pressed
        //set the speed to 0.5 to stop
        if(speed < 0.5 && !touchSensorLowLimit.getState())
            speed = 0.5;

        servoLeft.setPosition(speed);
        servoRight.setPosition(speed);
    }

}

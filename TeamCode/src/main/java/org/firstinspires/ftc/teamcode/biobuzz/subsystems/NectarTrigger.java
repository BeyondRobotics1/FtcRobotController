package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class NectarTrigger {
    Servo trigger;

    /**
     * Constructor
     * @param hardwareMap
     */
    public NectarTrigger (HardwareMap hardwareMap){
        trigger = hardwareMap.get(Servo.class, "nectarTrigger");
        trigger.setDirection(Servo.Direction.REVERSE);
        close();
    }

    /**
     * This function will close the trigger to hold balls
     * in the storage
     */
    public void open(){
        setPosition(0.0);//0.03
    }

    /**
     * This function will open the trigger to feed the balls
     * to the turret shooter
     */
    public void close(){

        setPosition(0.21); //0.24
    }

    public void setPosition(double position)
    {
        trigger.setPosition(position);
    }
}

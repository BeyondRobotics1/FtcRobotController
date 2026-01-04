package org.firstinspires.ftc.teamcode.decode.Subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Trigger {

    Servo trigger;

    /**
     * Constructor
     * @param hardwareMap
     */
    public Trigger (HardwareMap hardwareMap){
        trigger = hardwareMap.get(Servo.class, "trigger");
        //trigger.setDirection(Servo.Direction.REVERSE);

        close();

//        trigger.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * This function will close the trigger to hold balls
     * in the storage
     */
    public void open(){
        setPosition(0.00);//0.03
    }

    /**
     * This function will open the trigger to feed the balls
     * to the turret shooter
     */
    public void close(){
        setPosition(0.24);
    }

    public void setPosition(double position)
    {
        trigger.setPosition(position);//0.5
    }
}

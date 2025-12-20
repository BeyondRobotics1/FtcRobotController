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



    NormalizedColorSensor colorSensorHigh;
    Servo trigger;
    LinearOpMode mode;//set the telemetry
    float[] hsvValues;

    /**
     * Constructor
     * @param hardwareMap
     * @param mode
     */
    public Trigger (HardwareMap hardwareMap, LinearOpMode mode){
        this.mode = mode;
        trigger = hardwareMap.get(Servo.class, "trigger");
        //trigger.setDirection(Servo.Direction.REVERSE);

        close();

        hsvValues = new float[3];
        colorSensorHigh = hardwareMap.get(NormalizedColorSensor.class, "color2");

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

    public int getHighColor()
    {
        NormalizedRGBA colors = colorSensorHigh.getNormalizedColors();

        Color.colorToHSV(colors.toColor(), hsvValues);

        if(((DistanceSensor) colorSensorHigh).getDistance(DistanceUnit.CM) < 5) { //
            if (hsvValues[0] >= 120 && hsvValues[0] <= 200)
                return Color.GREEN;
                //mode.telemetry.addData("Color", "%s", "Green");
            else // (hsvValues[0] > 200 && hsvValues[0] < 250)
                return Color.BLUE;
                //mode.telemetry.addData("Color", "%s", "Purple");
        }
        else
            return Color.WHITE;
            //mode.telemetry.addData("Color", "%s", "White");
    }
}

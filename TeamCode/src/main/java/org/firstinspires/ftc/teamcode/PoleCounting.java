package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class PoleCounting {

    private DistanceSensor distanceSensor;
    private int polesToCount = 2;

    private double lastDistance = 322;
    private double currentDistance = 322;

    public double getCurrentDistance(DistanceUnit du){
        return distanceSensor.getDistance(du);
    }

    public PoleCounting ( int polesToCount, DistanceSensor distanceSensor){
            this.polesToCount = polesToCount;
            this.distanceSensor = distanceSensor;
    }

    public boolean polesReached()
    {
        ///
        boolean reached = false;


        return reached;
    }

    public double distanceToPole()
    {
        return currentDistance;
    }
}
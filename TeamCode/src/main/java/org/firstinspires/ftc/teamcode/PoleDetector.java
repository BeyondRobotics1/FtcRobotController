package org.firstinspires.ftc.teamcode;

import android.net.wifi.aware.IdentityChangedListener;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * PoleDetector Class uses distance sensor to count poles
 * Used to stop robot after the poles to detect count is reached
 */

public class PoleDetector {
    //poles to detect setting (defaults to third pole (high junction))
    private int polesToDetect = 0;
    //set up distance sensor
    private DistanceSensor distanceSensor;

    private double previousDistance = 0;
    private double currentDistance = 322;
    private double lastPoleDistance = 0;

    /**
     * constructor for distance sensor
     * @param distanceSensor distance sensor used for PoleDetector class
     */
    public PoleDetector (DistanceSensor distanceSensor){
        this.distanceSensor = distanceSensor;
    }
    /**
     *function that does the actual pole counting
     * @return The total number of poles detected
     */
    public int detectPoles()
    {
        currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);

        if(currentDistance <= 10 && currentDistance < previousDistance){//(previousDistance-20)
            polesToDetect++;

            lastPoleDistance = currentDistance;
        }

        previousDistance = currentDistance;

        return polesToDetect;
    }

    /**
     * Get the last pole's distance
     * @return the distance to the pole detected
     */
    public double getPoleDistance(){
        return lastPoleDistance;
    }

    /**
     * Get the current distance measurement
     * @return distance sensor's current measurement
     */
    public double getCurrentDistance()
    {
        return currentDistance;
    }
}

package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * The Claw class is to open and close the claw mechanism.
 * Our claw has two servos so that the slide can move smoothly.
 */
public class Claw {

    Servo claw;//left claw servo

    /** The colorSensor field will contain a reference to our color sensor hardware object */
    NormalizedColorSensor colorSensor;

    LinearOpMode mode;//set the telemetry

    boolean isClosed = false;

    /**
     * Constructor
     * @param hardwareMap: hardware map for finding claw servos
     * @param mode: for telemetry functions
     */
    public Claw (HardwareMap hardwareMap, LinearOpMode mode){

        this.mode = mode;
        claw = hardwareMap.get(Servo.class, "claw");


        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.
        //colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSlide");

        //claw1.setPosition(0.5);
        //claw2.setPosition(0.5);
    }

    /**
     * Set claw servo left and right positions
     * @param position: left servo target position

     */
    public void setPosition (double position){

        claw.setPosition(position);


        //mode.telemetry.addData("left claw position", claw1.getPosition());
        //mode.telemetry.addData("right claw position", claw2.getPosition());
    }

    /**
     * This function will close the claw
     */
    public void close(){
        setPosition(0.2);

        isClosed = true;
    }

    /**
     * This function will open the claw
     */
    public void open(){

        setPosition(1);

        isClosed = false;
    }

    /**
     *
     * @return true: claw is set to close position
     *         false: claw is set to open position
     */
    public boolean isClosed() {
        return isClosed;
    }

    /**
     *
     * @return: true is claw has cone, false, no cone
     */
    public boolean holdingCone()
    {
        if(colorSensor == null)
            return false;

        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        //use color sensor's alpha to detect if there is a cone or not
        return colors.alpha > 0.5;
    }
}

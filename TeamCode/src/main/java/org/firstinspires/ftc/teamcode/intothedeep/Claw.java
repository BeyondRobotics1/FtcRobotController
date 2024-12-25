package org.firstinspires.ftc.teamcode.intothedeep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

    Servo claw;//claw servo

    /** The colorSensor field will contain a reference to our color sensor hardware object */
    NormalizedColorSensor colorSensor;

    LinearOpMode mode;//set the telemetry

    enum ClawState
    {
        OPENED,
        CLOSED,
        NONE
    }

    ClawState state;

    //boolean isClosed = false;

    /**
     * Constructor
     * @param hardwareMap: hardware map for finding claw servos
     * @param mode: for telemetry functions
     */
    public Claw (HardwareMap hardwareMap, LinearOpMode mode){
        this.mode = mode;
        claw = hardwareMap.get(Servo.class, "claw");

//        claw.setDirection(Servo.Direction.REVERSE);

        state = ClawState.NONE;
    }
    public class closeClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            boolean finished = false;
            if (!finished) {
                close();
            }
            return finished;
        }
    }
    public class openClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            boolean finished = false;
            if (!finished) {
                open();
            }
            return finished;
        }
    }
    public Action openClaw(){
        return new openClaw();
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
    public void open(){

        //if(state != ClawState.OPENED) {
        setPosition(0.86);//0.7

        state = ClawState.OPENED;
        //}

    }

    /**
     * This function will open the claw
     */
    public void close(){

        //if(state != ClawState.CLOSED) {
            setPosition(0.52);//0.5
            state = ClawState.CLOSED;
        //}
    }

    /**
     *
     * @return true: claw is set to close position
     *         false: claw is set to open position
     */
    public boolean isClosed() {
        if(state == ClawState.CLOSED)
            return true;
        else
            return false;
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * The Claw class is to open and close the claw mechanism.
 * Our claw has two servos so that the slide can move smoothly.
 */
public class Claw {
    Servo claw1;//left claw servo
    Servo claw2;//right claw servo
    LinearOpMode mode;//set the telemetry

    /**
     * Constructor
      * @param hardwareMap: hardware map for finding claw servos
     * @param mode: for telemetry functions
     */
    public Claw (HardwareMap hardwareMap, LinearOpMode mode){

        this.mode = mode;
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");

        //claw1.setPosition(0.5);
        //claw2.setPosition(0.5);
    }

    /**
     * Set claw servo left and right positions
     * @param position1: left servo target position
     * @param position2: right servo target position
     */
    public void setPosition (double position1, double position2){

        claw1.setPosition(position1);
        claw2.setPosition(position2);

        //mode.telemetry.addData("left claw position", claw1.getPosition());
        //mode.telemetry.addData("right claw position", claw2.getPosition());
    }

    /**
     * This function will close the claw
     */
    public void close(){
        setPosition(0.42, 0.58);
    }

    /**
     * This function will open the claw
     */
    public void open(){
        setPosition(0.57, 0.43);
    }

}

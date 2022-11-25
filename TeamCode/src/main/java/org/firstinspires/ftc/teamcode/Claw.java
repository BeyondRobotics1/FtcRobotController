package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * The Claw class is to open and close the claw mechanism.
 * Our claw has two servos so that it is very powerful.
 */
public class Claw {

    Servo claw1;//left claw servo
    Servo claw2;//right claw servo

    LinearOpMode mode;//set the telemetry

    /**
     * Constructor
      * @param hardwareMap
     * @param mode
     */
    public Claw (HardwareMap hardwareMap, LinearOpMode mode){

        this.mode = mode;
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");

        claw1.setPosition(0.5);
        claw2.setPosition(0.5);
    }

    public void setPosition (double position1, double position2){

        claw1.setPosition(position1);
        claw2.setPosition(position2);

        mode.telemetry.addData("claw1 position", claw1.getPosition());
        mode.telemetry.addData("claw2 position", claw2.getPosition());

    }

}

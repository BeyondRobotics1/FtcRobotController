package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Indexer {
    private Servo indexer;
    private TouchSensor magnet;
    private LinearOpMode mode;

    private int current_index = 0;

    private double startingPosition = 0.005;//
    private double enqueueOnePosition = 0.377;
    private double enqueueTwoPosition = 0.755;

    public Indexer(HardwareMap hardwareMap, LinearOpMode linearOpMode) {
        this.mode = linearOpMode;
        indexer = hardwareMap.get(Servo.class, "indexer");

        ////to the middle range so servo and rotate either direction
        //indexer.setPosition(startingPosition);
        index(0);

        //magnet = hardwareMap.get(TouchSensor.class, "magnet");
        // indexer.setDirection(Servo.Direction.REVERSE);
    }

    public void reset()
    {
        setPosition(startingPosition);
    }

    public void setPosition(double position)
    {
        indexer.setPosition(position);
    }

    public double getPosition()
    {
        return indexer.getPosition();
    }

    public int getCurrent_index() { return current_index; }

    public void index(int number)
    {
        current_index = number;
        
        if (number == 1)
            setPosition(enqueueOnePosition);
        else if (number == 2)
            setPosition(enqueueTwoPosition);
        else {
            setPosition(startingPosition);
            current_index = 0;
        }
    }

}

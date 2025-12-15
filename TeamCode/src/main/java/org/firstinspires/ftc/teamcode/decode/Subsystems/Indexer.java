package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Indexer {
    private Servo indexer;
    private TouchSensor magnet;
    private LinearOpMode mode;

    private double startingPosition = 0.04;
    private double enqueueOnePosition = 0.45;
    private double enqueueTwoPosition = 0.8;

    public Indexer(HardwareMap hardwareMap, LinearOpMode linearOpMode) {
        this.mode = linearOpMode;
        indexer = hardwareMap.get(Servo.class, "indexer");

        //to the middle range so servo and rotate either direction
        indexer.setPosition(startingPosition);

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

    public void queue(int number)
    {
        if (number == 1)
            setPosition(enqueueOnePosition);
        else if (number == 2)
            setPosition(enqueueTwoPosition);
        else
            setPosition(startingPosition);
    }


}

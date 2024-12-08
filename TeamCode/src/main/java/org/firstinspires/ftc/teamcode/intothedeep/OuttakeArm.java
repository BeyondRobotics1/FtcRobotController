package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeArm {

    //For auto, starting position for specimen
    private final double AUTO_SPECIMEN_SCORE_POSITION = 0.6;

    //For auto and teleop, position to pickup specimen from human player
    public final double SAMPLE_PICKUP_POSITION = 1;
    public final double SPECIMEN_PICKUP_POSITION = 0;
    public final double SPECIMEN_SCORE_POSITION = 0.65;
    public final double SAMPLE_DELIVERY_POSITION = 0.2;

    private Servo rotateServo;
    private  LinearOpMode mode;//set the telemetry

    public OuttakeArm(HardwareMap hardwareMap, LinearOpMode mode)
    {
        this.mode = mode;
        rotateServo = hardwareMap.get(Servo.class, "outtakeArm");
    }

    public void Rotate(double position)
    {
        rotateServo.setPosition(position);
    }
}

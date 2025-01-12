package org.firstinspires.ftc.teamcode.intothedeep.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeArm {

    //For auto and teleop, position to pickup specimen from human player
    public final double SAMPLE_PICKUP_POSITION = 0;//1;
    public final double SPECIMEN_PICKUP_POSITION = 0.85;//0.82;
    public final double SPECIMEN_READY_POSITION = 0.59;//0.7

    //this position to align with high chamber
    public final double SPECIMEN_SHUFFLE_POSITION = 0.38; //

    //this position to score a specimen
    public final double SPECIMEN_SCORE_POSITION = 0.25; //


    public final double SAMPLE_DELIVERY_POSITION = 0.625; //0.75

    public final double SPECIMEN_PARK_POSITION1 = 0.33; //0.40
    public final double SPECIMEN_PARK_POSITION2 = 0.3; //0.36

    private Servo rotateServo;
    private Servo rotateServo2;
    private LinearOpMode mode;//set the telemetry

    public OuttakeArm(HardwareMap hardwareMap, LinearOpMode mode)
    {
        this.mode = mode;
        rotateServo = hardwareMap.get(Servo.class, "outtakeArm");
        rotateServo2 = hardwareMap.get(Servo.class, "outtakeArm2");

        rotateServo.setDirection(Servo.Direction.REVERSE);

        //axon servo, may give 30 degree more of rotation
        //((ServoImplEx) intakeServo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));
        //((ServoImplEx) rotateServo).setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void RotateTo(double position)
    {
        rotateServo.setPosition(position);
        rotateServo2.setPosition(position);
    }
}

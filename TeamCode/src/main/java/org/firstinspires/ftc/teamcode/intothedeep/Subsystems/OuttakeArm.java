package org.firstinspires.ftc.teamcode.intothedeep.Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeArm {

    //For auto and teleop, position to pickup specimen from human player
    public final double SAMPLE_PICKUP_POSITION = 0;//1;
    public final double SPECIMEN_PICKUP_POSITION = 1;//0;
    public final double SPECIMEN_READY_POSITION = 0.7;
    public final double SPECIMEN_SCORE_POSITION = 0.32; //
    public final double SAMPLE_DELIVERY_POSITION = 0.75; //0.22

    private Servo rotateServo;
    private LinearOpMode mode;//set the telemetry

    public OuttakeArm(HardwareMap hardwareMap, LinearOpMode mode)
    {
        this.mode = mode;
        rotateServo = hardwareMap.get(Servo.class, "outtakeArm");

        rotateServo.setDirection(Servo.Direction.REVERSE);

        //axon servo, may give 30 degree more of rotation
        //((ServoImplEx) intakeServo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));
        //((ServoImplEx) rotateServo).setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void Rotate(double position)
    {
        rotateServo.setPosition(position);
    }
}

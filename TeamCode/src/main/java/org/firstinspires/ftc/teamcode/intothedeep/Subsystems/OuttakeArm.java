package org.firstinspires.ftc.teamcode.intothedeep.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeArm {

    //For auto and teleop, position to pickup specimen from human player
    public final double SAMPLE_PICKUP_POSITION = 1;
    public final double SPECIMEN_PICKUP_POSITION = 0;
    public final double SPECIMEN_READY_POSITION = 0.3;
    public final double SPECIMEN_SCORE_POSITION = 0.68; //0.75
    //public final double SAMPLE_DELIVERY_POSITION = 0.3; //0.22

    private Servo rotateServo;
    private LinearOpMode mode;//set the telemetry

    public class autoToSpecimenScore implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            boolean finished = false;
            if(!finished){
                Rotate(SPECIMEN_SCORE_POSITION);
            }
            return finished;
        }
    }
    public Action autoToSpecimenScore(){
        return new OuttakeArm.autoToSpecimenScore();
    }
    public class autoToSpecimenPick implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            boolean finished = false;
            if(!finished){
                Rotate(SPECIMEN_PICKUP_POSITION);
            }
            return finished;
        }
    }
    public Action autoToSpecimenPick(){
        return new OuttakeArm.autoToSpecimenPick();
    }

    public OuttakeArm(HardwareMap hardwareMap, LinearOpMode mode)
    {
        this.mode = mode;
        rotateServo = hardwareMap.get(Servo.class, "outtakeArm");

        //axon servo, may give 30 degree more of rotation
        //((ServoImplEx) intakeServo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));
        //((ServoImplEx) rotateServo).setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void Rotate(double position)
    {
        rotateServo.setPosition(position);
    }
}

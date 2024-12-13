package org.firstinspires.ftc.teamcode.intothedeep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.centerstage.picasso.Outtake;

public class OuttakeArm {

    //For auto, starting position for specimen
    private final double AUTO_SPECIMEN_SCORE_POSITION = 0.6;

    //For auto and teleop, position to pickup specimen from human player
    public final double SAMPLE_PICKUP_POSITION = 1;
    public final double SPECIMEN_PICKUP_POSITION = 0;
    public final double SPECIMEN_READY_POSITION = 0.65;
    public final double SPECIMEN_SCORE_POSITION = 0.75;
    public final double SAMPLE_DELIVERY_POSITION = 0.22;

    private Servo rotateServo;
    private  LinearOpMode mode;//set the telemetry

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
    }

    public void Rotate(double position)
    {
        rotateServo.setPosition(position);
    }
}

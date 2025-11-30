package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Indexer {
    private CRServo indexer;
    private TouchSensor magnet;
    private LinearOpMode mode;
    public Indexer(HardwareMap hardwareMap, LinearOpMode linearOpMode) {
        this.mode = linearOpMode;
        indexer = hardwareMap.get(CRServo.class, "indexer");
        magnet = hardwareMap.get(TouchSensor.class, "magnet");
        // indexer.setDirection(Servo.Direction.REVERSE);
    }
    public void startTurn(int di){
        // on button pressed
        if (di == 1) {
            indexer.setPower(0.9);
        }
        else{
            indexer.setPower(0.1);
        }
    }
    public boolean checkForStop(){
        // loop on button release
        if(magnet.isPressed()){
            indexer.setPower(0.5);
            return true;
        }
        else{
            return false;
        }
    }
}

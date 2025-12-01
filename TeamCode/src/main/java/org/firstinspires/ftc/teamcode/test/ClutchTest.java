package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Concept: Clutch test", group = "Concept")
public class ClutchTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        Servo clutch = hardwareMap.get(Servo.class, "clutchTest");
        waitForStart();
        clutch.setPosition(0.5);
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()){
            if(gamepad1.aWasPressed()){
                clutch.setPosition(0.8);
            }else if(gamepad1.aWasReleased()){
                clutch.setPosition(0.3);
            }
        }

    }
}

package org.firstinspires.ftc.teamcode.tutorial.Michael;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Servo OpMode", group = "Michael")
@Disabled
public class ServoGamepadOpMode extends OpMode {
    ProgrammingBoard1 servo = new ProgrammingBoard1();

    @Override
    public void init(){
        servo.init(hardwareMap);
    }

    @Override
    public void loop(){
        if(gamepad1.a){
            servo.setServoPosition(1.0);
        }
        else if(gamepad1.b){
            servo.setServoPosition(0.5);
        }
        else {
            servo.setServoPosition(0.0);
        }
    }
}

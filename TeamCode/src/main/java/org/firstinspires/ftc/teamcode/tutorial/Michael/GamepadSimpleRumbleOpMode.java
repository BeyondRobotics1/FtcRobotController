package org.firstinspires.ftc.teamcode.tutorial.Michael;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Simple Rumble OpMode", group = "Michael")
@Disabled
public class GamepadSimpleRumbleOpMode extends OpMode {
    @Override
    public void init(){

    }

    @Override
    public void loop(){
        if(gamepad1.a){
            gamepad1.rumble(100);
        }
    }
}

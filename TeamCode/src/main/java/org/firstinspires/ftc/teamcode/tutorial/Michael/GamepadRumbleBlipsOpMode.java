package org.firstinspires.ftc.teamcode.tutorial.Michael;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Blips OpMode", group = "Michael")
@Disabled
public class GamepadRumbleBlipsOpMode extends OpMode {
    boolean wasA;

    public void init(){

    }

    @Override
    public void loop(){
        if(gamepad1.a && !wasA){
            gamepad1.rumbleBlips(3);
        }
        wasA = gamepad1.a;
    }
}

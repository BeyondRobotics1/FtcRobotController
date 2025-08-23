package org.firstinspires.ftc.teamcode.tutorial.Michael;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Toggle", group = "Michael")
@Disabled
public class ToggleOpMode extends OpMode {
    ProgrammingBoard1 toggle = new ProgrammingBoard1();
    boolean aNotPressed;
    boolean motorOff;

    @Override
    public void init(){
        toggle.init(hardwareMap);
    }

    @Override
    public void loop(){
        if(gamepad1.a && !aNotPressed){
            motorOff = !motorOff;
            telemetry.addData("Motor", motorOff);

            if(motorOff){
                toggle.setMotor(0.5);
            } else {
                toggle.setMotor(0.0);
            }
        }
        aNotPressed = gamepad1.a;
    }
}

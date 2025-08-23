package org.firstinspires.ftc.teamcode.tutorial.Michael;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Motor OpMode", group = "Michael")
@Disabled
public class MotorOpMode extends OpMode {
    ProgrammingBoard1 board = new ProgrammingBoard1();

    @Override
    public void init(){
        board.init(hardwareMap);
    }

    @Override
    public void loop(){
        board.setMotor(-gamepad1.left_stick_y);
    }
}

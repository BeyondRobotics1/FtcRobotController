package org.firstinspires.ftc.teamcode.tutorial.Michael;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp (name = "Touch Sensor OpMode", group = "Michael")
@Disabled
public class TouchSensorOpMode extends OpMode {
    ProgrammingBoard1 board = new ProgrammingBoard1();

    @Override
    public void init(){
        board.init(hardwareMap);
    }

    @Override
    public void loop(){
        telemetry.addData("Touch Sensor State", board.getTouchSensorState());
        telemetry.addData("Touch Sensor Pressed", board.isPressed());
    }
}

package org.firstinspires.ftc.teamcode.tutorial.Michael;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name = "Heading OpMode", group = "Michael")
@Disabled
public class HeadingOpMode extends OpMode {
    ProgrammingBoard1 heading = new ProgrammingBoard1();

    @Override
    public void init(){
        heading.init(hardwareMap);
    }

    @Override
    public void loop(){
        telemetry.addData("Our heading", heading.getHeading(AngleUnit.DEGREES));

        if(gamepad1.a){
            heading.resetHeading();
        }
    }
}

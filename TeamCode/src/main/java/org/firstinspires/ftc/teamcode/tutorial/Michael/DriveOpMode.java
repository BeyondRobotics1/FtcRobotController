package org.firstinspires.ftc.teamcode.tutorial.Michael;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Drive", group = "Michael")
public class DriveOpMode extends OpMode {
    MecanumDrive mecanumDrive;

    @Override
    public void init(){
        mecanumDrive = new MecanumDrive();
        mecanumDrive.init(hardwareMap, true);
    }

    @Override
    public void loop(){
        if(gamepad1.left_bumper){
            mecanumDrive.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            mecanumDrive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
        telemetry.addData("Heading", mecanumDrive.getHeading());
    }
}

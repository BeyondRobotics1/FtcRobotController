package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@TeleOp(name="Test2", group="Linear Opmode")
@Disabled
public class Test2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx slideMotor1 =  hardwareMap.get(DcMotorEx.class, "slide1"); //hardwareMap.get(DcMotor.class, "slide1");
        DcMotorEx slideMotor2 =  hardwareMap.get(DcMotorEx.class, "slide2"); //hardwareMap.get(DcMotor.class, "slide1");

        slideMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.left_bumper) {
                slideMotor1.setPower(-gamepad1.left_stick_y);
            }

            if(gamepad2.left_bumper) {
                slideMotor2.setPower(-gamepad2.left_stick_y);
            }


            //sleep(50);
        }
    }
}
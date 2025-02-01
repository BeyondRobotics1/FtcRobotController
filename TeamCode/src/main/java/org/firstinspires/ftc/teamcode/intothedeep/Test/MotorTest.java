package org.firstinspires.ftc.teamcode.intothedeep.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Motor Test", group = "Test")
public class MotorTest extends LinearOpMode {

    //DcMotorEx motor;
    DcMotorEx slideMotor1;
    DcMotorEx slideMotor2;

    double current1 = 0, current2 = 0;
    double maxCurrent1 = 0, maxCurrent2 = 0;
    double power = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        //motor = hardwareMap.get(DcMotorEx.class, "test");

        slideMotor1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slideMotor2 = hardwareMap.get(DcMotorEx.class, "slide2");

        slideMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        //wait for play button clicked
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            power = -gamepad2.left_stick_y;
            slideMotor1.setPower(power);
            slideMotor2.setPower(power);

            //motor.setPower(-gamepad2.left_stick_y);

            current1 = slideMotor1.getCurrent(CurrentUnit.AMPS);
            current2 = slideMotor2.getCurrent(CurrentUnit.AMPS);

            if(current1 > maxCurrent1)
                maxCurrent1 = current1;
            if(current2 > maxCurrent2)
                maxCurrent2 = current2;

            telemetry.addData("current1" , current1);
            telemetry.addData("current2" , current2);
            telemetry.addData("max current1", maxCurrent1);
            telemetry.addData("max current2", maxCurrent2);

            telemetry.update();

        }
    }


}

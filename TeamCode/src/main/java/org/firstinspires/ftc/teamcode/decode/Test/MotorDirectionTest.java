package org.firstinspires.ftc.teamcode.decode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.decode.Subsystems.Trigger;

@TeleOp(name = "Flywheel Motor Direction Test", group = "Decode Test")
public class MotorDirectionTest extends LinearOpMode {

    private DcMotorEx leftFlywheel, rightFlywheel;

    @Override
    public void runOpMode() throws InterruptedException {

        String motorName = "";

        telemetry.addData("gamepad1.a", "Left flywheel." );
        telemetry.addData("gamepad1.b", "Right flywheel." );
        telemetry.addData("gamepad1.x", "Both flywheels." );
        telemetry.addData("gamepad1.left_stick_y", "Power the motor" );
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            if (gamepad1.aWasPressed()) {
                if(leftFlywheel == null) {
                    leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
                    //leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
                    motorName = "left flywheel";
                }

                rightFlywheel = null;
            }
            else if (gamepad1.bWasPressed()) {
                if(rightFlywheel == null) {
                    rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
                    rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
                    motorName = "right flywheel";
                }

                leftFlywheel = null;
            }
            else if (gamepad1.xWasPressed()) {

                if(leftFlywheel == null) {
                    leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
                    //leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
                }

                if(rightFlywheel == null) {
                    rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
                    rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
                }

                motorName = "both flywheels";
            }

            double power = gamepad1.left_stick_y;


            if(leftFlywheel != null) {
                leftFlywheel.setPower(power);
                telemetry.addData("Left motor velocity", "%s",  leftFlywheel.getVelocity());
            }

            if(rightFlywheel != null) {
                rightFlywheel.setPower(power);
            }

            telemetry.addData("Motor(s) Selected", "%s",  motorName);
            telemetry.addData("Motor Power", "%.5f",  power);
            telemetry.update();
        }
    }
}

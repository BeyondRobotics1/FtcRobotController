package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Outtake Servo Test", group = "Into the Deep")
public class OuttakeServoTest extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo   servo = null;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;

    @Override
    public void runOpMode() throws InterruptedException {

        //change servo name to find it
        //servo = hardwareMap.get(Servo.class, "outtakeRotateClaw");;

        // Wait for the start button

        telemetry.addData("gamepad1.a", "Outtake Rotate Claw Servo." );
        telemetry.addData("gamepad1.x", "Outtake Arm Servo 1." );
        telemetry.addData("gamepad1.y", "Outtake Arm Servo 2." );
        telemetry.addData("gamepad1.b", "Claw Servo." );
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            if(servo == null) {
                if (gamepad1.a)
                    servo = hardwareMap.get(Servo.class, "outtakeRotateClaw");
                else if (gamepad1.x)
                    servo = hardwareMap.get(Servo.class, "outtakeArm");
                else if (gamepad1.y)
                    servo = hardwareMap.get(Servo.class, "outtakeArm");
                else if (gamepad1.b)
                    servo = hardwareMap.get(Servo.class, "claw");
            }

            if(servo != null) {
                //use gamepad1 left trigger to set servo positions dynamically
                double pivotPosition = gamepad1.left_trigger;
                telemetry.addData("Servo position", Math.abs(pivotPosition));
                servo.setPosition(pivotPosition);
            }

            telemetry.update();

        }

    }
}

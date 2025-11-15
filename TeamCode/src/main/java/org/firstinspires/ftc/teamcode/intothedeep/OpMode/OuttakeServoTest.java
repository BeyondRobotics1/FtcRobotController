package org.firstinspires.ftc.teamcode.intothedeep.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Outtake Servo Test", group = "Into the Deep")
@Disabled
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

        telemetry.addData("gamepad2.a", "Outtake Rotate Claw Servo." );
        telemetry.addData("gamepad2.x", "Outtake Arm Servo 1." );
        telemetry.addData("gamepad2.y", "Outtake Arm Servo 2." );
        telemetry.addData("gamepad2.b", "Claw Servo." );
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        String servoName = "";
        while (!isStopRequested() && opModeIsActive()) {
            if(servo == null) {
                if (gamepad2.a) {
                    servo = hardwareMap.get(Servo.class, "outtakeRotateClaw");
                    servoName = "ClawRotor";
                }
                else if (gamepad2.x) {
                    servo = hardwareMap.get(Servo.class, "outtakeArm");
                    servo.setDirection(Servo.Direction.REVERSE);
                    servoName = "OuttakeArm";
                }
                else if (gamepad2.y) {
                    servo = hardwareMap.get(Servo.class, "outtakeArm2");
                    servoName = "OuttakeArm2";
                }
                else if (gamepad2.b) {
                    servo = hardwareMap.get(Servo.class, "claw");
                    servoName = "Claw";
                }
            }

            if(servo != null) {
                //use gamepad1 left trigger to set servo positions dynamically
                double pivotPosition = gamepad1.left_trigger;
                telemetry.addData(servoName+ " position", Math.abs(pivotPosition));
                servo.setPosition(pivotPosition);
            }

            telemetry.update();

        }

    }
}

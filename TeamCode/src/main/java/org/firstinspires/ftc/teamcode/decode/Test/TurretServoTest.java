package org.firstinspires.ftc.teamcode.decode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.decode.Subsystems.Trigger;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Turret;

@TeleOp(name = "Turret Servo Test", group = "Decode Test")
public class TurretServoTest extends LinearOpMode {
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   100;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo   servoLeft, servoRight;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = false;


    @Override
    public void runOpMode() {

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servoLeft = hardwareMap.get(Servo.class, "turretLeft");
        servoRight = hardwareMap.get(Servo.class, "turretRight");

        servoLeft.setDirection(Servo.Direction.REVERSE);
        servoRight.setDirection(Servo.Direction.REVERSE);

        servoLeft.setPosition(0.5);
        servoRight.setPosition(0.5);

        // Wait for the start button
        telemetry.addData("Servo Initial Position", "%5.2f", position);
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();

        boolean point_of_interest = false;

        // Scan servo till stop pressed.
        while(opModeIsActive()){

            point_of_interest = false;

            // slew the servo, according to the rampUp (direction) variable.
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT ;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction

                    point_of_interest = true;
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
                if (position <= MIN_POS ) {

                    point_of_interest = true;

                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }

            if(Math.abs(position - 0.5) < 0.001)
                point_of_interest = true;

            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            servoLeft.setPosition(position);
            servoRight.setPosition(position);

            if(!point_of_interest)
                sleep(CYCLE_MS);
            else
                sleep(5000);

            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}

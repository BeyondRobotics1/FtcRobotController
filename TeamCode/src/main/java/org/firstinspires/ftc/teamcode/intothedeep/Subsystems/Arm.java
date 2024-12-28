package org.firstinspires.ftc.teamcode.intothedeep.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.Helper;

public class Arm {
    enum ArmMode
    {
        AUTO_UP,
        AUTO_DOWN,
        AUTO_STAY,
        MANUAL
    }

    static final double     COUNTS_PER_MOTOR_REV    = 1993.6;// Gobilda 71.2:1, 84 RPM
    static final double     DRIVE_GEAR_REDUCTION    = 60.0/48.0 ;   // No External Gearing.
    static final double     COUNTS_PER_DEGREE       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / 360.0;

    //intake, outtake, specimen, hang angle in degrees
    //arm can rotate to
    enum ArmTargetAngle
    {
        INTAKE(0),
        OUTTAKE(1),
        SPECIMEN(2),
        HANG(3),
        MANUAL(4);

        private final int value;
        private ArmTargetAngle(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }
    //the arm angle in degrees corresponding to the above
    //predefined position
    final double[] TARGET_ANGLES = {0, -89, -50, -90, 0};

    int autoTargetPosition = 0;

    ArmMode activeMode = ArmMode.AUTO_STAY;

    DcMotorEx armMotor1;
    //DcMotorEx armMotor2;

    LinearOpMode mode;

    public Arm(HardwareMap hardwareMap, LinearOpMode mode) {
        this.mode = mode;

        armMotor1 = hardwareMap.get(DcMotorEx.class, "arm1");
        //armMotor2 = hardwareMap.get(DcMotorEx.class, "arm2");

        //armMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        //armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        //When there is no power, we want the motor to hold the position
        armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Set motors encoder position to 0, then set motor to RUN_USING_ENCODER
     */
    public void runWithEncoder()
    {
        //When the motor is in this mode it would turn the encoder position back to 0. The motor will stop.
        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //In the mode, the Control Hub uses the encoder to manage the motor's speed.
        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * rotate arm using RUN_TO_POSITION mode,
     * This method will wait for target position is reached
     * @param angle: angle rotates to in degree
     * @param speed: power for motors
     */
    public void rotateTo(double angle, double speed)
    {
        // Determine new target position, and pass to motor controller
        int newPosition = (int)(angle * COUNTS_PER_DEGREE);
        armMotor1.setTargetPosition(newPosition);
        //armMotor2.setTargetPosition(newPosition);

        // Turn On RUN_TO_POSITION
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor1.setPower(speed);
        //armMotor2.setPower(speed);

        // keep looping while we are still active and both motors are running.
        while (armMotor1.isBusy()){
            //if(touchSensorHighLimit.getState() == false) {
            //   break;
            //}
        }

        // Stop all motion
        armMotor1.setPower(0);
        //armMotor2.setPower(0);

        // Turn off RUN_TO_POSITION
        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /**
     * rotate to predefined angle
     * @param predefinedAngle: predefined angle
     * @param speed: motor power
     */
    public void rotateToTargetAngle(ArmTargetAngle predefinedAngle, double speed)
    {
        rotateTo(TARGET_ANGLES[predefinedAngle.getValue()], speed);
    }

    /**
     * Rotate to specific angle without waiting for motor to finish
     * @param angle: angle rotates to in degree
     * @param speed: motor power
     */
    public void rotateToWithoutWaiting(double angle, double speed)
    {
        //calc the target position
        autoTargetPosition = (int)(angle * COUNTS_PER_DEGREE);

        int currentPosition = armMotor1.getCurrentPosition();

        //if the target position and current position are the same
        //Set the current state as AUTO_STAY
        if(autoTargetPosition == currentPosition) {
            activeMode = ArmMode.AUTO_STAY;
            return;
        }
        else if(autoTargetPosition > currentPosition) //move up
        {
            //Set the current state as AUTO_UP
            activeMode = ArmMode.AUTO_UP;
        }
        else { //move down
            //Set the current state as AUTO_DOWN
            activeMode = ArmMode.AUTO_DOWN;
        }

        //set target position
        armMotor1.setTargetPosition(autoTargetPosition);
        //armMotor2.setTargetPosition(autoTargetPosition);

        //set motor as RUN_TO_POSITION
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Power on
        armMotor1.setPower(speed);
        //armMotor2.setPower(speed);
    }

    /**
     * rotate to predefined angle without waiting for motor to finish
     * @param angle: predefined angle
     * @param speed: motor power
     */
    public void rotateToTargetAngleWithoutWaiting(ArmTargetAngle angle, double speed)
    {
        rotateToWithoutWaiting(TARGET_ANGLES[angle.getValue()], speed);
    }

    /**
     * Call this function in op loop to stop motors if position
     * is reached or touch sensor is pressed down
     */
    public void autoRotateToWithoutWaitingLoop() {
        //
        if(activeMode == ArmMode.AUTO_UP ||
                activeMode == ArmMode.AUTO_DOWN) {

            int currentPosition = armMotor1.getCurrentPosition();
            boolean targetPositionReached = false;

            //mode.telemetry.addData("Target position", autoTargetPosition);
            //mode.telemetry.addData("current position", currentPosition);
            //mode.telemetry.addData("High sensor pressed", !touchSensorHighLimit.getState());
            //mode.telemetry.addData("Low sensor pressed", !touchSensorLowLimit.getState());
            //mode.telemetry.addData("mode", activeMode);
            //mode.telemetry.update();

            //slide is moving up
            //reached the target position or
            //pressed down the upper limit touch sensor
            if (activeMode == ArmMode.AUTO_UP)
            {
                if(currentPosition >= autoTargetPosition)// || !touchSensorHighLimit.getState())
                    targetPositionReached = true;
            }
            //slide is moving down
            //reached the target position or
            //pressed down the low limit touch sensor
            else if (currentPosition <= autoTargetPosition){// || !touchSensorLowLimit.getState()) {
                targetPositionReached = true;
            }

            //reach the target position
            //stop motors
            if(targetPositionReached)
            {
                activeMode = ArmMode.AUTO_STAY;

                armMotor1.setPower(0);
                //armMotor2.setPower(0);

                armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }


    public void setPower(double power)
    {
        activeMode = ArmMode.MANUAL;

        //set the motors to RUN_USING_ENCODER if not yet
        if(armMotor1.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)// ||
                //armMotor2.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
        {
            armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        double localPower = Helper.squareWithSign(power);//Helper.cubicWithSign(power);//

//        //slide move down
//        if(localPower < -0.01) {
//            //low limit touch sensor is pushed
//            if (!touchSensorLowLimit.getState()) {
//                localPower = 0;
//
//                //reset the slide 0 position
//                //we want to do this only once
//                if(!resetSlideDone)
//                {
//                    runWithEncoder();
//                    resetSlideDone = true;
//                }
//            }
//        }
//        else if(localPower > 0.01) { //slide move up
//            //high limit touch sensor is pushed
//            if (!touchSensorHighLimit.getState())
//                localPower = 0;
//        }

        armMotor1.setPower(localPower);
        //armMotor2.setPower(localPower);
    }

    /**
     * Get the arm's angle in degrees
     * @return arm's angle in degrees
     */
    public double getArmAngle()
    {
        return armMotor1.getCurrentPosition() / COUNTS_PER_DEGREE;
    }

}

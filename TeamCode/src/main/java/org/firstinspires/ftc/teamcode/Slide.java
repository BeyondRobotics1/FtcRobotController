package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Slide class models robot's slide mechanism which includes two motors and two touch sensors
 */
public class Slide {
    static final double     COUNTS_PER_MOTOR_REV    = 291.76;//Rev Motor: 10.43 gear ratio; Gobilda 537.7
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;   // No External Gearing.
    static final double     PULLEY_DIAMETER_INCHES  = 1.404 ; // spool wheel inches
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (PULLEY_DIAMETER_INCHES * 3.1415);

    //ground, low, medium, high junction heights
    //slide can move to
    double junctionPoleHeights[] = {0, 14.5, 24, 34};

    //two motors powering the slide
    DcMotorEx slideMotor1;
    DcMotorEx slideMotor2;

    //Digital touch sensors used to stop the slide moving up/down too much
    DigitalChannel touchSensorLowLimit;
    DigitalChannel touchSensorHighLimit;

    /**
     * Constructor
     * @param hardwareMap: used for sensor, motor, and telemetry
     */
    public Slide(HardwareMap hardwareMap){

        //low and high limit touch sensor
        touchSensorLowLimit =  hardwareMap.get(DigitalChannel.class, "limit_low");
        touchSensorLowLimit.setMode(DigitalChannel.Mode.INPUT);
        touchSensorHighLimit =  hardwareMap.get(DigitalChannel.class, "limit_high");
        touchSensorHighLimit.setMode(DigitalChannel.Mode.INPUT);

        //we used two motors to power the slide
        slideMotor1 =  hardwareMap.get(DcMotorEx.class, "slide1"); //hardwareMap.get(DcMotor.class, "slide1");
        slideMotor2 =  hardwareMap.get(DcMotorEx.class, "slide2"); //hardwareMap.get(DcMotor.class, "slide2");

        slideMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        //When there is no power, we want the motor to hold the position
        slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Set motors encoder position to 0, then set motor to RUN_USING_ENCODER
     */
    public void runWithEncoder()
    {
        //When the motor is in this mode it would turn the encoder position back to 0. The motor will stop.
        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //In the mode, the Control Hub uses the encoder to manage the motor's speed.
        slideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Move slide up/down using RUN_TO_POSITION mode,
     * This method will wait for target position is reached
     * @param inches: inches to move up
     * @param speed: power for motors
     */
    public void moveTo(double inches, double speed)
    {
        // Determine new target position, and pass to motor controller
        int newPosition = (int)(inches * COUNTS_PER_INCH);
        slideMotor1.setTargetPosition(newPosition);
        slideMotor2.setTargetPosition(newPosition);

        // Turn On RUN_TO_POSITION
        slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideMotor1.setPower(speed);
        slideMotor2.setPower(speed);

        // keep looping while we are still active and both motors are running.
        while (slideMotor1.isBusy()){
            //if(touchSensorHighLimit.getState() == false) {
            //   break;
            //}
        }

        // Stop all motion
        slideMotor1.setPower(0);
        slideMotor2.setPower(0);

        // Turn off RUN_TO_POSITION
        slideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Move slide up/down using RUN_TO_POSITION mode
     * This method will not wait for motors to reach the target position
     * @param inches: inches to move up
     * @param speed: power for motors
     */
    public void moveToWithoutWaiting(double inches, double speed)
    {
        // Determine new target position, and pass to motor controller
        int newPosition = (int)(inches * COUNTS_PER_INCH);
        slideMotor1.setTargetPosition(newPosition);
        slideMotor2.setTargetPosition(newPosition);

        // Turn On RUN_TO_POSITION
        slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideMotor1.setPower(speed);
        slideMotor2.setPower(speed);
    }

    /**
     * Move to specific junction position
     * @param junction: 0 - ground, 1 - low, 2 - medium, 3 - high junction
     * @param speed: motor power
     */
    public void moveToJunction(int junction, double speed)
    {
        if(junction < 0 || junction > 3)
            return;

        moveTo(junctionPoleHeights[junction], speed);
    }

    /**
     * Move slide up and down
     * @param power negative move down, positive move up
     */
    public void setPower(double power)
    {
        //set the motors to RUN_USING_ENCODER if not yet
        if(slideMotor1.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
        {
            slideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        double localPower = Helper.squareWithSign(power);

        //slide move down
        if(power < -0.01) {
            //low limit touch sensor is pushed
            if (touchSensorLowLimit.getState() == false)
                localPower = 0;
        }
        else if(power > 0.01) { //slide move up
            //high limit touch sensor is pushed
            if (touchSensorHighLimit.getState() == false)
                localPower = 0;
        }

        slideMotor1.setPower(localPower);
        slideMotor2.setPower(localPower);
    }

    /**
     * Reset the slide to the ground position
     * it also reset the motors' encoder position back to 0
     */
    public void resetSlide()
    {
        //set the motors to RUN_USING_ENCODER, otherwise, motor may not move
        if(slideMotor1.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
        {
            slideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        slideMotor1.setPower(-0.5);
        slideMotor2.setPower(-0.5);

        //check if the slide reach the ground by checking
        //low stop limit touch sensor
        while(touchSensorLowLimit.getState())
            ;
        runWithEncoder();
    }

    /**
     * Get the slide height in inches
     * @return slide position in inches
     */
    public double getSlideHeightInches()
    {
        return slideMotor1.getCurrentPosition() / COUNTS_PER_INCH;
    }

    /**
     *
     * @param low, true get the low limit touch sensor, otherwise, high limit
     * @return: touch sensor state
     */
    public boolean getTouchSensorState(boolean low)
    {
        if(low)
            return touchSensorLowLimit.getState();
        else
            return  touchSensorHighLimit.getState();
    }
}

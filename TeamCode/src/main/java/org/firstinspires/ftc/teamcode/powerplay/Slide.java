package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.common.Helper;

/**
 * Slide class models robot's slide mechanism which includes two motors and two touch sensors
 */
public class Slide {
    static final double     COUNTS_PER_MOTOR_REV    = 291.76;//Rev Motor: 10.43 gear ratio; Gobilda 537.7
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;   // No External Gearing.
    static final double     PULLEY_DIAMETER_INCHES  = 1.404 ; // spool wheel inches
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (PULLEY_DIAMETER_INCHES * 3.1415);

    //slide positions before grabbing the cones
    static final double coneStackHeights[] = {5.2, 3.8, 2.4, 1.2, 0};
    //slide positions after grabbing the cones
    static final double coneLiftHeights[] = {11, 9.6, 8.2, 7.5, 7.5};
    //distance for pole to cone stack
    static final double moveFromPole[] = {13.9, 14.1, 14.1, 14.1, 14};//{13.7, 13.9, 13.9, 13.9, 13.8}

    //ground, low, medium, high junction heights
    //slide can move to
    double junctionPoleHeights[] = {0, 15.2, 24.2, 34};//{0, 14.2, 24, 33.8}

    enum SlideMode
    {
        AUTO_UP,
        AUTO_DOWN,
        AUTO_STAY,
        MANUAL;
    }
    int autoTargetPosition = 0;
    SlideMode activeMode = SlideMode.AUTO_STAY;


    //two motors powering the slide
    DcMotorEx slideMotor1;
    DcMotorEx slideMotor2;

    //Digital touch sensors used to stop the slide moving up/down too much
    DigitalChannel touchSensorLowLimit;
    DigitalChannel touchSensorHighLimit;

    // The IMU sensor object
    // We will use its pitch angle for pitching control
    IMU imu = null;

    LinearOpMode mode;

    //slide 0 position is reset
    boolean resetSlideDone = false;

    /**
     * Constructor
     * @param hardwareMap: used for sensor, motor, and telemetry
     */
    public Slide(HardwareMap hardwareMap, LinearOpMode mode){
        this.mode = mode;

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
     * set the slide imu
     * @param imu
     */
    public void setImu(IMU imu){
        this.imu = imu;
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
     * Move to specific position without waiting for motor to finish
     * @param inches: inches to move
     * @param speed: motor power
     */
    public void moveToWithoutWaiting(double inches, double speed)
    {
        //calc the target position
        autoTargetPosition = (int)(inches * COUNTS_PER_INCH);

        int currentPosition = slideMotor1.getCurrentPosition();

        //if the target position and current position are the same
        //Set the current state as AUTO_STAY
        if(autoTargetPosition == currentPosition) {
            activeMode = SlideMode.AUTO_STAY;
            return;
        }
        else if(autoTargetPosition > currentPosition) //move up
        {
            //Set the current state as AUTO_UP
            activeMode = SlideMode.AUTO_UP;
        }
        else { //move down
            //Set the current state as AUTO_DOWN
            activeMode = SlideMode.AUTO_DOWN;
        }

        //set target position
        slideMotor1.setTargetPosition(autoTargetPosition);
        slideMotor2.setTargetPosition(autoTargetPosition);

        //set motor as RUN_TO_POSITION
        slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Power on
        slideMotor1.setPower(speed);
        slideMotor2.setPower(speed);
    }

    /**
     * Move to specific junction position without waiting for motor to finish
     * @param speed: motor power
     */
    public void scoreToJunctionWithoutWaiting(double speed)
    {
        double currentSlideHeight = getSlideHeightInches();

        if(currentSlideHeight > junctionPoleHeights[3] - 2) //high junction
            moveToWithoutWaiting(junctionPoleHeights[3] - 2 ,speed);
        else if(currentSlideHeight > junctionPoleHeights[2] - 2.5 &&
                currentSlideHeight < junctionPoleHeights[2] + 2) //medium junction
            moveToWithoutWaiting(junctionPoleHeights[2] - 2 ,speed);
        else if (currentSlideHeight > junctionPoleHeights[1] - 2.5 &&
                currentSlideHeight < junctionPoleHeights[1] + 2) //low junction
            moveToWithoutWaiting(junctionPoleHeights[1] - 2.5 ,speed);
    }

    /**
     * Move to specific junction position without waiting for motor to finish
     * @param junction: 0 - ground, 1 - low, 2 - medium, 3 - high junction
     * @param speed: motor power
     */
    public void moveToJunctionWithoutWaiting(int junction, double speed)
    {
        //junction id should be 0, 1, 2, or 3
        if(junction < 0 || junction > 3)
            return;

        moveToWithoutWaiting(junctionPoleHeights[junction], speed);
    }

    /**
     * Call this function in op loop to stop motors if position
     * is reached or touch sensor is pressed down
     */
    public void autoMoveToWithoutWaitingLoop() {
        //
        if(activeMode == SlideMode.AUTO_UP ||
                activeMode == SlideMode.AUTO_DOWN) {

            int currentPosition = slideMotor1.getCurrentPosition();
            boolean targetPositionReached = false;

//            mode.telemetry.addData("Target position", autoTargetPosition);
//            mode.telemetry.addData("current position", currentPosition);
//            mode.telemetry.addData("High sensor pressed", !touchSensorHighLimit.getState());
//            mode.telemetry.addData("Low sensor pressed", !touchSensorLowLimit.getState());
//            mode.telemetry.addData("mode", activeMode);
//            mode.telemetry.update();

            //slide is moving up
            //reached the target position or
            //pressed down the upper limit touch sensor
            if (activeMode == SlideMode.AUTO_UP)
            {
                if(currentPosition >= autoTargetPosition || !touchSensorHighLimit.getState())
                    targetPositionReached = true;
            }
            //slide is moving down
            //reached the target position or
            //pressed down the low limit touch sensor
            else if (currentPosition <= autoTargetPosition || !touchSensorLowLimit.getState()) {
                targetPositionReached = true;
            }

            //reach the target position
            //stop motors
            if(targetPositionReached)
            {
                activeMode = SlideMode.AUTO_STAY;

                slideMotor1.setPower(0);
                slideMotor2.setPower(0);

                slideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    /**
     * Move slide up and down manually
     * @param power negative move down, positive move up
     */
    public void setPower(double power)
    {
        activeMode = SlideMode.MANUAL;

        //set the motors to RUN_USING_ENCODER if not yet
        if(slideMotor1.getMode() != DcMotor.RunMode.RUN_USING_ENCODER ||
                slideMotor2.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
        {
            slideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        double localPower = Helper.squareWithSign(power);//Helper.cubicWithSign(power);//

//        //Anti-tipping control kicks in when our side's height is bigger than 20 inches
//        if (imu != null && getSlideHeightInches() > 20) { //
//            //get the pitch angle of IMU
//            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//            double pitchAngle = orientation.getPitch(AngleUnit.DEGREES);
//
//            //if pitch angle is greater than 3 degrees, dangerous
//            if (Math.abs(pitchAngle) >= 2.5) {
//                //move slide up to level our robot
//                localPower = 0.8;
//
//                mode.telemetry.addData("local power after", localPower);
//                mode.telemetry.addData("pitch angle", pitchAngle);
//                mode.telemetry.update();
//            }
//        }

        //slide move down
        if(localPower < -0.01) {
            //low limit touch sensor is pushed
            if (!touchSensorLowLimit.getState()) {
                localPower = 0;

                //reset the slide 0 position
                //we want to do this only once
                if(!resetSlideDone)
                {
                    runWithEncoder();
                    resetSlideDone = true;
                }
            }
        }
        else if(localPower > 0.01) { //slide move up
            //high limit touch sensor is pushed
            if (!touchSensorHighLimit.getState())
                localPower = 0;
        }

        slideMotor1.setPower(localPower);
        slideMotor2.setPower(localPower);
    }

//    /**
//     * Reset the slide to the ground position
//     * it also reset the motors' encoder position back to 0
//     */
//    public void resetSlide()
//    {
//        activeMode = SlideMode.MANUAL;
//
//        //set the motors to RUN_USING_ENCODER, otherwise, motor may not move
//        if(slideMotor1.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
//        {
//            slideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//
//        slideMotor1.setPower(-0.5);
//        slideMotor2.setPower(-0.5);
//
//        //check if the slide reach the ground by checking
//        //low stop limit touch sensor
//        while(touchSensorLowLimit.getState())
//            ;
//        runWithEncoder();
//    }

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

//    private void move(double power)
//    {
//        double localPower = power;
//
//        //slide move down
//        if(power < -0.01) {
//            //low limit touch sensor is pushed
//            if (!touchSensorLowLimit.getState())
//                localPower = 0;
//        }
//        else if(power > 0.01) { //slide move up
//            //high limit touch sensor is pushed
//            if (!touchSensorHighLimit.getState())
//                localPower = 0;
//        }
//
//        slideMotor1.setPower(localPower);
//        slideMotor2.setPower(localPower);
//    }
}

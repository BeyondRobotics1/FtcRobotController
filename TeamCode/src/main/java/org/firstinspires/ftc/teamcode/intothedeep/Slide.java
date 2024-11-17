package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.Helper;

public class Slide{

    static final double     COUNTS_PER_MOTOR_REV    = 145.1;//1160 //384.5;// Gobilda 13.7:1, 435 RPM
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;   // No External Gearing.
    static final double     PULLEY_DIAMETER_INCHES  = 1.404 ; // spool wheel inches
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (PULLEY_DIAMETER_INCHES * 3.1415);

    //predefined position
    //slide can move to
    enum SlideTargetPosition
    {
        DOWN(0),
        INTAKE(1),//max extention during intake
        INTAKE1(2),
        INTAKE2(3),
        LOW_BASKET(4),
        HIGH_BASkET(5),
        SPECIMEN_PICKUP(6),
        SPECIMEN_DELIVERY(7),
        MANUAL(8);

        private final int value;
        private SlideTargetPosition(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }
    //the slide extension length in inches corresponding to the above
    //predefined position
    //we can extend 16 inches to keep in 42 size limit
    double[] slidePositionInches = {0, 12, 4, 8, 5.0, 21.0, 0, 8.0, 0};//28

    enum SlideMode
    {
        AUTO_UP,
        AUTO_DOWN,
        AUTO_STAY,
        MANUAL
    }
    int autoTargetPosition = 0;
    org.firstinspires.ftc.teamcode.intothedeep.Slide.SlideMode activeMode = org.firstinspires.ftc.teamcode.intothedeep.Slide.SlideMode.AUTO_STAY;

    Arm arm;
    DcMotorEx slideMotor1;
    DcMotorEx slideMotor2;

    LinearOpMode mode;

    public Slide(HardwareMap hardwareMap, LinearOpMode mode, Arm arm) {
        this.mode = mode;
        this.arm  = arm;

        slideMotor1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slideMotor2 = hardwareMap.get(DcMotorEx.class, "slide2");

        slideMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

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
     * Move to specific junction position
     * @param position: predefined position
     * @param speed: motor power
     */
    public void moveToPredefinedPosition(SlideTargetPosition position, double speed)
    {

        moveTo(slidePositionInches[position.getValue()], speed);
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
            activeMode = org.firstinspires.ftc.teamcode.intothedeep.Slide.SlideMode.AUTO_STAY;
            return;
        }
        else if(autoTargetPosition > currentPosition) //move up
        {
            //Set the current state as AUTO_UP
            activeMode = org.firstinspires.ftc.teamcode.intothedeep.Slide.SlideMode.AUTO_UP;
        }
        else { //move down
            //Set the current state as AUTO_DOWN
            activeMode = org.firstinspires.ftc.teamcode.intothedeep.Slide.SlideMode.AUTO_DOWN;
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
     * Move to specific position without waiting for motor to finish
     * @param position: predefined position
     * @param speed: motor power
     */
    public void moveToPredefinedPositionWithoutWaiting(SlideTargetPosition position, double speed)
    {

        moveToWithoutWaiting(slidePositionInches[position.getValue()], speed);
    }

    /**
     * Call this function in op loop to stop motors if position
     * is reached or touch sensor is pressed down
     */
    public void autoMoveToWithoutWaitingLoop() {
        //
        if(activeMode == org.firstinspires.ftc.teamcode.intothedeep.Slide.SlideMode.AUTO_UP ||
                activeMode == org.firstinspires.ftc.teamcode.intothedeep.Slide.SlideMode.AUTO_DOWN) {

            int currentPosition = slideMotor1.getCurrentPosition();
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
            if (activeMode == org.firstinspires.ftc.teamcode.intothedeep.Slide.SlideMode.AUTO_UP)
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
                activeMode = org.firstinspires.ftc.teamcode.intothedeep.Slide.SlideMode.AUTO_STAY;

                slideMotor1.setPower(0);
                slideMotor2.setPower(0);

                slideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public void setPower(double power)
    {
        activeMode = org.firstinspires.ftc.teamcode.intothedeep.Slide.SlideMode.MANUAL;

        //set the motors to RUN_USING_ENCODER if not yet
        if(slideMotor1.getMode() != DcMotor.RunMode.RUN_USING_ENCODER ||
                slideMotor2.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
        {
            slideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        mode.telemetry.addData("arm angle", "%f", arm.getArmAngle());
        mode.telemetry.addData("slide extension", "%f", getSlideHeightInches());
        //mode.telemetry.update();

        //if slide is moving up and arm is angled less than 30 degrees and we are about to exceed the size constraint, stop the slide
        //reached robot horizontal limit
        //set the power to 0 and stop the slide
//        if(power >= 0.05 && Math.abs(arm.getArmAngle()) <= 30 &&
//                getSlideHeightInches() >= slidePositionInches[SlideTargetPosition.INTAKE.getValue()]-2)
//            power = 0;



        double localPower = Helper.squareWithSign(power);//Helper.cubicWithSign(power);//

//        //slide move down
//        if(localPower < -0.01) {
//            //low limit touch sensor is
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

        slideMotor1.setPower(localPower);
        slideMotor2.setPower(localPower);

    }

    /**
     * Get the slide height in inches
     * @return slide position in inches
     */
    public double getSlideHeightInches()
    {
        return slideMotor1.getCurrentPosition() / COUNTS_PER_INCH;
    }
}

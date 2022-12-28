package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slide {
    static final double     COUNTS_PER_MOTOR_REV    = 291.76;//Rev Motor: 10.43 gear ratio; 537.7 ; //Rev Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;   // No External Gearing.
    static final double     PULLEY_DIAMETER_INCHES  = 1.404 ; // spool wheel inches
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (PULLEY_DIAMETER_INCHES * 3.1415);

    //ground, low, medium, high junction heights
    //slide can move to
    double junctionPoleHeights[] = {0, 13, 24, 33.3};

    int zeroPosition = 0;
    DcMotorEx slideMotor1;
    DcMotorEx slideMotor2;

    //A digital touch sensor used to stop the slide moving up/down too much
    DigitalChannel touchSensorLowLimit;
    DigitalChannel touchSensorHighLimit;

    public Slide(HardwareMap hardwareMap){
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

    public void slideRunWithEncorder()
    {
        //When the motor is in this mode it would turn the encoder position back to 0. The motor will stop.
        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //In the mode, the Control Hub uses the encoder to manage the motor's speed.
        slideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        zeroPosition = slideMotor1.getCurrentPosition();

    }

    public void moveTo(double inches, double speed)
    {
        int newPosition = (int)(zeroPosition + inches * COUNTS_PER_INCH);

        slideMotor1.setTargetPosition(newPosition);
        slideMotor2.setTargetPosition(newPosition);

        slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideMotor1.setPower(speed);
        slideMotor2.setPower(speed);

        while (slideMotor1.isBusy()){
            //if(touchSensorLowLimit.getState() == false ||
            //    touchSensorHighLimit.getState() == false) {
            //   break;
            //}
        }
        slideMotor1.setPower(0);
        slideMotor2.setPower(0);

        slideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //get the slide height in inches
    public double getSlideHeightInches()
    {
        return slideMotor1.getCurrentPosition() / COUNTS_PER_INCH;
    }

    /**
     * Move slide up and down
     * @param power negative move down, positive move up
     */
    public void setPower(double power)
    {
        double localPower = power;//Helper.squareWithSign(power);

        //slide move down
        if(power < -0.01) {
            if (touchSensorLowLimit.getState() == false) //touch sensor is pushed
            {
                localPower = 0;
            }
        }
        else if(power > 0.01) { //slide move up
            if (touchSensorHighLimit.getState() == false) //touch sensor is pushed
                localPower = 0;
        }

        slideMotor1.setPower(localPower);
        slideMotor2.setPower(localPower);
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

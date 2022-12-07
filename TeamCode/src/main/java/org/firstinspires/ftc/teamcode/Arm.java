package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Arm class is to control the arm, claw, and slide.
 */
public class Arm {

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ; //goBilda Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;   // No External Gearing.
    static final double     PULLEY_DIAMETER_INCHES  = 1.404 ; // spool wheel inches
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                            (PULLEY_DIAMETER_INCHES * 3.1415);

    //
    enum SlidePosition
    {
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }

    //slide position
    SlidePosition slidePosition = SlidePosition.GROUND;


    //turret position, 0-left, 1 - middle, 2 - right
    int turretPosition = 1;

    //The left, middle, and right position for arm servo
    //0.265, 0.165, 0.065
    double turretServoPositions[] = {0.267, 0.167, 0.063};
    DistanceSensor distanceSensor;

    Servo servoArm;
    Servo servoClaw;

    DcMotor slideMotor1;
    DcMotor slideMotor2;

    //A digital touch sensor used to stop the slide moving up/down too much
    DigitalChannel touchSensorLowLimit;
    DigitalChannel touchSensorHighLimit;

    public Arm(HardwareMap hardwareMap)
    {
        //servos
        servoArm = hardwareMap.get(Servo.class,"arm");
        //servoClaw = hardwareMap.get(Servo.class,"claw");


        distanceSensor = hardwareMap.get(DistanceSensor.class, "dsSlide");

        //we used two motors to power the slide
        slideMotor1 = hardwareMap.get(DcMotor.class, "slide1");
        slideMotor2 = hardwareMap.get(DcMotor.class, "slide2");

        //When there is no power, we want the motor to hold the position
        slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        touchSensorLowLimit =  hardwareMap.get(DigitalChannel.class, "limit_low");
        touchSensorLowLimit.setMode(DigitalChannel.Mode.INPUT);

        touchSensorHighLimit =  hardwareMap.get(DigitalChannel.class, "limit_high");
        touchSensorHighLimit.setMode(DigitalChannel.Mode.INPUT);
    }


    /**
     * Set turret to left, middle, or right
     * @param turretPosition: 0 - left, 1 - middle, 2 -right
     */
    public void setTurretPosition(int turretPosition)
    {

        if (this.turretPosition != turretPosition) {

            if (turretPosition == 1 && this.turretPosition == 0)
                servoArm.setPosition(0.164);
            else
                servoArm.setPosition(turretServoPositions[turretPosition]);

            this.turretPosition = turretPosition;
        }

    }


    //get the turret servo's position
    public double getTurretPosition(){

        return servoArm.getPosition();

    }

    //Distance sensor
    public double getDistanceINCH(){
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    //set the turret servo's position
    public void setTurretPosition(double position){
        servoArm.setPosition(position);
    }

    /**
     * Move slide up and down
     * @param power negative move down, positive move up
     */
    public void setSlidePower(double power)
    {
        double localPower = power;

        //slide move down
        if(power < -0.01) {
            if (touchSensorLowLimit.getState() == false) //touch sensor is pushed
            {
                localPower = 0;
                slidePosition = SlidePosition.GROUND;
            }
        }
        else if(power > 0.01) { //slide move up
            if (touchSensorHighLimit.getState() == false) //touch sensor is pushed
            {
                localPower = 0;
                slidePosition = SlidePosition.HIGH;
            }
        }

        slideMotor1.setPower(localPower);
        slideMotor2.setPower(localPower);
    }

    //
    public double getSlideMotorCurrentPosition()
    {
        return slideMotor1.getCurrentPosition();
    }

    //
    public boolean getTouchSensorState(boolean low)
    {
        if(low)
            return touchSensorLowLimit.getState();
        else
            return  touchSensorHighLimit.getState();
    }
}

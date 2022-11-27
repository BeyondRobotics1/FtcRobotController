package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Arm class is to control the arm, claw, and slide.
 */
public class Arm {

    int arm_position_id = 1;

    //The left, middle, and right position for arm servo
    //0.265, 0.165, 0.065
    double arm_positions[] = {0.267, 0.167, 0.063};
    //DistanceSensor distanceSensor;

    Servo servoArm;
    Servo servoClaw;

    DcMotor slideMotor1;
    DcMotor slideMotor2;

    public Arm(HardwareMap hardwareMap)
    {
        //servos
        servoArm = hardwareMap.get(Servo.class,"arm");
        //servoClaw = hardwareMap.get(Servo.class,"claw");


        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

        //we used two motors to power the slide
        slideMotor1 = hardwareMap.get(DcMotor.class, "slide1");
        slideMotor2 = hardwareMap.get(DcMotor.class, "slide2");

        //When there is no power, we want the motor to hold the position
        slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    //
    public void setTurretPosition(int arm_position_id)
    {
        if(this.arm_position_id != arm_position_id) {

            if(arm_position_id == 1 && this.arm_position_id == 0)
                servoArm.setPosition(0.164);
            else
                servoArm.setPosition(arm_positions[arm_position_id]);

            this.arm_position_id = arm_position_id;
        }
    }


    //get the turret servo's position
    public double getTurretPosition(){

        return servoArm.getPosition();

    }
    //Distance sensor
    public double getDistanceINCH(){
        //return distanceSensor.getDistance(DistanceUnit.INCH);
        return 0;
    }

    //set the turret servo's position
    public void setTurretPosition(double position){

        servoArm.setPosition(position);

    }

    public void moveSlide(double power)
    {
        slideMotor1.setPower(Helper.cubicWithSign(power));
        slideMotor2.setPower(Helper.cubicWithSign(power));
    }
}

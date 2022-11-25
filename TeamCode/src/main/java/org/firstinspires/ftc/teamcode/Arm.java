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

    int arm_position_id = 0;

    //The left, middle, and right position for arm servo
    double arm_positions[] = {0.265, 0.165, 0.065};
    DistanceSensor distanceSensor;
    Servo servoArm;
    Servo servoClaw;

    //we used SparkMini control for our prototype robot
    //because we can not get a REV expansion hub
    //
    //DcMotorSimple slideMotor;

    DcMotor slideMotor1;
    DcMotor slideMotor2;

    public Arm(HardwareMap hardwareMap)
    {
        //servos
        servoArm = hardwareMap.get(Servo.class,"arm");
        //servoClaw = hardwareMap.get(Servo.class,"claw");

        //slideMotor = hardwareMap.get(DcMotorSimple.class, "slide");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

        //we used two motors to power the slide
        slideMotor1 = hardwareMap.get(DcMotor.class, "slide1");
        slideMotor2 = hardwareMap.get(DcMotor.class, "slide2");

        //When there is no power, we want the motor to hold the position
        slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //servoClaw.scaleRange(0.1, 0.25);
        //servoClaw.setPosition(0);

        //servoArm.scaleRange(0.15, 0.85);
        //servoArm.setPosition(1);
    }

    public void openClaw()
    {
        servoClaw.setPosition(0);
    }

    public void closeClaw()
    {
        servoClaw.setPosition(1.0);
    }

    //
    public void setClawPosition(int arm_position_id)
    {
        this.arm_position_id = arm_position_id;

        servoArm.setPosition(arm_positions[this.arm_position_id]);
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

    public void moveSlide(double power)
    {
        //slideMotor.setPower(Helper.cubicWithSign(power));
        slideMotor1.setPower(Helper.cubicWithSign(power));
        slideMotor2.setPower(Helper.cubicWithSign(power));
    }
}

package org.firstinspires.ftc.teamcode.tutorial.Michael;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ProgrammingBoard1{
    private DigitalChannel touchSensor;
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private Servo servo;
    private DistanceSensor distanceSensor;
    private ColorSensor colorSensor;
    private IMU imu;

    public void init (HardwareMap hardwareMap){
        touchSensor = hardwareMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        motorLeft = hardwareMap.get(DcMotor.class, "motor1");
        motorRight = hardwareMap.get(DcMotor.class, "motor2");
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = hardwareMap.get(Servo.class, "servo");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientation));
    }
    public boolean getTouchSensorState(){
        return touchSensor.getState();
    }

    public boolean isPressed(){
        /*if(touchSensor.getState() == false){
            return true;
        }
        else {
            return false;
        }*/
        return !touchSensor.getState();
    }

    public void setMotor(double power){
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    public void setServoPosition(double position){
        servo.setPosition(position);
    }

    public int getColor(int color){
        if(color == 0){
            return colorSensor.red();
        }
        else if(color == 1) {
            return colorSensor.blue();
        }
        else {
            return colorSensor.green();
        }
    }

    public double getDistance(DistanceUnit distanceUnit){
        return distanceSensor.getDistance(distanceUnit);
    }

    public double getHeading(AngleUnit angleUnit){
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    public void resetHeading(){
        imu.resetYaw();
    }
}

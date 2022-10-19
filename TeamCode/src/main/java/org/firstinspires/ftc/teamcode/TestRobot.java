package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TestRobot {

    public TestRobot(HardwareMap hwMap){

        touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        colorSensor = hwMap.get(ColorSensor.class, "arm_color_sensor");
        distanceSensor = hwMap.get(DistanceSensor.class, "arm_distance_sensor");
        motor = hwMap.get(DcMotor.class, "arm_motor");
        servo = hwMap.get(Servo.class, "arm_servo");

        slideMotor = hwMap.get(DcMotorSimple.class, "slide_motor");

    }

    public void init(){

        touchSensor.setMode(DigitalChannel.Mode.INPUT);


        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        startMotorPosition = motor.getCurrentPosition();

        setSlideMotorSpeed(0);
    }

    public boolean getTouchSensorState(){
        //get the state of the touch sensor, pressed or released?
        return touchSensor.getState();

    }
    public int getAmountRed(){

        return colorSensor.red();

    }
    public int getAmountBlue(){

        return colorSensor.blue();

    }
    public double getDistanceCM(){

        return distanceSensor.getDistance(DistanceUnit.CM);

    }
    public void setMotorSpeed(double speed){

        motor.setPower(speed);

    }

    public double getMotorTurns()
    {
        return (motor.getCurrentPosition() - startMotorPosition)/COUNTS_PER_MOTOR_REV;
    }
    public void setServoPosition(double position){

        servo.setPosition(position);

    }

    public void setSlideMotorSpeed(double speed)
    {
        slideMotor.setPower(speed);
    }

    private DigitalChannel touchSensor;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private DcMotor motor;
    private Servo servo;
    private DcMotorSimple slideMotor;

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_CM   = 9.6 ;     // GoBILDA Mecanum
    static final double     COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);

    double startMotorPosition;
}
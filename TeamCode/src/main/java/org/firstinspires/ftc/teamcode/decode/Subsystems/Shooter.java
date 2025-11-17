package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private Motor leftFlywheel;
    private Motor rightFlywheel;
    private LinearOpMode mode;


    private MotorGroup flyWheel;
    public static double kP = 20;
    public static double kV = 0.7;

    public Shooter(HardwareMap hardwareMap, LinearOpMode linearOpMode)
    {
        this.mode = linearOpMode;
        //leftFlywheel = hardwareMap.get(DcMotor.class, "leftFlywheel");
        //rightFlywheel = hardwareMap.get(DcMotor.class, "rightFlywheel");
        leftFlywheel = new Motor(hardwareMap, "leftFlywheel", Motor.GoBILDA.BARE);
        rightFlywheel = new Motor(hardwareMap, "rightFlywheel", Motor.GoBILDA.BARE);

        leftFlywheel.setInverted(true);

        leftFlywheel.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        flyWheel = new MotorGroup( leftFlywheel, rightFlywheel);

        flyWheel.setRunMode(Motor.RunMode.VelocityControl);
        flyWheel.setVeloCoefficients(0.8, 0.01, 0);
        flyWheel.setFeedforwardCoefficients(0, 0.8);//0.7
    }

    public void shoot()
    {
        flyWheel.set(0.43);
        //setPower(0.7);
    }

    public void setPower(double power)
    {
        leftFlywheel.motor.setPower(power);
        rightFlywheel.motor.setPower(power);
    }

    public void stop()
    {
        leftFlywheel.motor.setPower(0);
        rightFlywheel.motor.setPower(0);
    }
}

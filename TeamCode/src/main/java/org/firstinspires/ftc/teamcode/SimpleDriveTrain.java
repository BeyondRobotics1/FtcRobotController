package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.Helper;

public class SimpleDriveTrain {
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;
    double yPowerScale = 0.9;
    double xPowerScale = 0.9;
    double rxPowerScale = 0.65;
    public SimpleDriveTrain(HardwareMap hardwareMap, LinearOpMode mode) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void setMotorPower(double v, double v2, double v3, double v4){
        leftFront.setPower(v);
        rightFront.setPower(v2);
        leftBack.setPower(v3);
        rightBack.setPower(v4);
    }
    public void driveRobotCentric(double leftStickY, double leftStickX, double rightStickX){
        double y = leftStickY;
        double x = leftStickX;
        double rx = rightStickX * rxPowerScale; //Helper.squareWithSign(right_stick_x);

        if(x != 0.0 && Math.abs(y/x) >= 1.2)
            x = 0.0;

        if(y != 0.0 && Math.abs(x/y) >= 1.2)
            y = 0.0;

        y *= yPowerScale;//Helper.squareWithSign(left_stick_y); // Remember, this is reversed!
        x *= xPowerScale;//Helper.squareWithSign(left_stick_x * 1.1); // Counteract imperfect strafing

        x = Helper.cubicWithSign(x);//squareWithSign(x);
        y = Helper.cubicWithSign(y);//squareWithSign(y);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y - x - rx) / denominator;
        double backLeftPower = (y + x - rx) / denominator;
        double frontRightPower = (y + x + rx) / denominator;
        double backRightPower = (y - x + rx) / denominator;

        setMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
}

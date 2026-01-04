package org.firstinspires.ftc.teamcode.decode.Subsystems;



import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.Helper;
import org.firstinspires.ftc.teamcode.common.PID;


public class DriveTrain {


    // The IMU sensor object
    IMU imu = null;

    //Use DcMotorEx to support bulk read.
    DcMotorEx motorFrontLeft;
    DcMotorEx motorBackLeft;
    DcMotorEx motorFrontRight;
    DcMotorEx motorBackRight;

    LinearOpMode mode;

    private final ElapsedTime runtime = new ElapsedTime();


    double motorFrontLeftMaxCurrent = 0;
    double motorBackLeftMaxCurrent = 0;
    double motorFrontRightMaxCurrent = 0;
    double motorBackRightMaxCurrent = 0;
    double motorTotalMaxCurrent = 0;

    //for teleop
    //adjust forward/backward, left/right, and rotation power
    private double y_power_scale = 0.9; //forward/backward power adjustment
    private double x_power_scale = 0.9; //left/right power adjustment, make it slower
    private double rx_power_scale = 0.85;//rotation power adjustment, make it slower


    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     MOVE_FORWARD_ADJUSTMENT = 0.95; //adjustment moving forward auto
    static final double     MOVE_BACKWARD_ADJUSTMENT = 0.95; //adjustment moving backward auto
    static final double     MOVE_LEFT_ADJUSTMENT = 1.1; //adjustment moving backward auto

    static final double     COUNTS_PER_MOTOR_REV    = 384.5;// 435 rmp; 537.7, 312 rmp ;    // eg: goBilda Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.7795 ;     // goBilda Mecanum wheel inches
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_DEGREE         = COUNTS_PER_INCH / 4.7;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    /**
     * Constructor
     * @param hardwareMap: for hardware lookup
     * @param mode: for telemetry
     */
    public DriveTrain(HardwareMap hardwareMap, LinearOpMode mode, boolean hasIMU)
    {

        this.mode = mode;

        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "rightFront");

        //Reverse motors
        //motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);


        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    //robot centric
    //https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
    public void setPower(double left_stick_y,
                         double left_stick_x,
                         double right_stick_x) {
        //
        double y = left_stick_y;
        double x = left_stick_x;
        double rx = right_stick_x * rx_power_scale; //Helper.squareWithSign(right_stick_x);

        if(x != 0.0 && Math.abs(y/x) >= 1.2)
            x = 0.0;

        if(y != 0.0 && Math.abs(x/y) >= 1.2)
            y = 0.0;

        y *= y_power_scale;//Helper.squareWithSign(left_stick_y); // Remember, this is reversed!
        x *= x_power_scale;//Helper.squareWithSign(left_stick_x * 1.1); // Counteract imperfect strafing

        x = Helper.cubicWithSign(x);//squareWithSign(x);
        y = Helper.cubicWithSign(y);//squareWithSign(y);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        setMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    //field centric
    //https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
    public void setPower2(double left_stick_y,
                          double left_stick_x,
                          double right_stick_x,
                          double botHeading) {

        //
        double y = left_stick_y * y_power_scale;
        double x = left_stick_x * x_power_scale;
        double rx = right_stick_x * rx_power_scale; //Helper.squareWithSign(right_stick_x);

        //Read inverse IMU heading, as the IMU heading is CW positive
        double rotX = Helper.cubicWithSign(x * Math.cos(-botHeading) - y * Math.sin(-botHeading));
        double rotY = Helper.cubicWithSign(x * Math.sin(-botHeading) + y * Math.cos(-botHeading));

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = (rotY  + rotX  + rx) / denominator;
        double backLeftPower = (rotY  - rotX  + rx) / denominator;
        double frontRightPower = (rotY  - rotX  - rx) / denominator;
        double backRightPower = (rotY  + rotX  - rx) / denominator;

        setMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }


    //set power for each motors
    public void setMotorPower(double frontLeft, double frontRight,
                              double backLeft, double backRight)
    {
        motorFrontLeft.setPower(frontLeft);
        motorFrontRight.setPower(frontRight);
        motorBackLeft.setPower(backLeft);
        motorBackRight.setPower(backRight);
    }

    public void printMotorCurrents()
    {
        double motorFrontLeftCurrent = motorFrontLeft.getCurrent(CurrentUnit.AMPS);
        double motorBackLeftCurrent = motorBackLeft.getCurrent(CurrentUnit.AMPS);
        double motorFrontRightCurrent = motorFrontRight.getCurrent(CurrentUnit.AMPS);
        double motorBackRightCurrent = motorBackRight.getCurrent(CurrentUnit.AMPS);

        if (motorFrontLeftCurrent > motorFrontLeftMaxCurrent)
            motorFrontLeftMaxCurrent = motorFrontLeftCurrent;

        if (motorBackLeftCurrent > motorBackLeftMaxCurrent)
            motorBackLeftMaxCurrent = motorBackLeftCurrent;

        if (motorFrontRightCurrent > motorFrontRightMaxCurrent)
            motorFrontRightMaxCurrent = motorFrontRightCurrent;

        if (motorBackRightCurrent > motorBackRightMaxCurrent)
            motorBackRightMaxCurrent = motorBackRightCurrent;


        double total = motorFrontLeftCurrent + motorBackLeftCurrent +
                motorFrontRightCurrent + motorBackRightCurrent;

        if(total > motorTotalMaxCurrent)
            motorTotalMaxCurrent = total;

        mode.telemetry.addData("motorFrontLeft", "%.2f", motorFrontLeftCurrent);
        mode.telemetry.addData("motorBackLeft", "%.2f", motorBackLeftCurrent);
        mode.telemetry.addData("motorFrontRight", "%.2f", motorFrontRightCurrent);
        mode.telemetry.addData("motorBackRight", "%.2f", motorBackRightCurrent);

        mode.telemetry.addData("motorFrontLeftMax", "%.2f", motorFrontLeftMaxCurrent);
        mode.telemetry.addData("motorBackLeftMax", "%.2f", motorBackLeftMaxCurrent);
        mode.telemetry.addData("motorFrontRightMax", "%.2f", motorFrontRightMaxCurrent);
        mode.telemetry.addData("motorBackRightMax", "%.2f", motorBackRightMaxCurrent);

        mode.telemetry.addData("total", "%.2f", total);
        mode.telemetry.addData("totalMax", "%.2f", motorTotalMaxCurrent);

    }
}

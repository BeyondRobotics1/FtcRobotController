package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class DriveTrain {

    DistanceSensor mrOds = null;

    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;

    LinearOpMode mode;

    private ElapsedTime runtime = new ElapsedTime();
    private int lfPos;
    private int rfPos;
    private int lrPos;
    private int rrPos;
    private double lineThreshold = 0.7; // floor should be below this value, line above
    private double redThreshold = 1.9; // red should be below this value, blue above


    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: goBilda Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.7795 ;     // goBilda Mecanum wheel inches
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                        (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_DEGREE         = COUNTS_PER_INCH / 4;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    public DriveTrain(HardwareMap hardwareMap, LinearOpMode mode)
    {

        this.mode = mode;

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
       /* motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);*/



        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runWithEncoder()
    {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void startRunToPosition()
    {
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void stopRunToPosition()
    {
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(double left_stick_y,
                         double left_stick_x,
                         double right_stick_x) {

            double y = Helper.cubicWithSign(left_stick_y); // Remember, this is reversed!
            double x = Helper.cubicWithSign(left_stick_x * 1.1); // Counteract imperfect strafing
            double rx = Helper.cubicWithSign(right_stick_x);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = Helper.cubicWithSign(-(y + x + rx) / denominator);
            double backLeftPower = Helper.cubicWithSign((y - x + rx) / denominator);
            double frontRightPower = Helper.cubicWithSign((y - x - rx) / denominator);
            double backRightPower = Helper.cubicWithSign(-(y + x - rx) / denominator);

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }

    // Distances in inches, angles in deg, speed 0.0 to 0.6
    // move forward
    public void moveLeft(int howMuch, double speed) {
        // howMuch is in inches. A negative howMuch moves backward.

        // fetch motor positions
        lfPos = motorFrontLeft.getCurrentPosition();
        rfPos = motorFrontRight.getCurrentPosition();
        lrPos = motorBackLeft.getCurrentPosition();
        rrPos = motorBackRight.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * COUNTS_PER_INCH;
        rfPos += howMuch * COUNTS_PER_INCH;
        lrPos += howMuch * COUNTS_PER_INCH;
        rrPos += howMuch * COUNTS_PER_INCH;

        // move robot to new position
        motorFrontLeft.setTargetPosition(lfPos);
        motorFrontRight.setTargetPosition(rfPos);
        motorBackLeft.setTargetPosition(lrPos);
        motorBackRight.setTargetPosition(rrPos);

        startRunToPosition();

        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackLeft.setPower(speed);
        motorBackRight.setPower(speed);

        // wait for move to complete
        while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() && motorBackRight.isBusy()) {

            // Display it for the driver.
            mode.telemetry.addLine("Move Foward");
            mode.telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            mode.telemetry.addData("Actual", "%7d :%7d", motorFrontLeft.getCurrentPosition(),
                    motorFrontRight.getCurrentPosition(), motorBackLeft.getCurrentPosition(),
                    motorBackRight.getCurrentPosition());
            mode.telemetry.update();
        }

        // Stop all motion;
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        stopRunToPosition();
    }

    public void moveForward(int howMuch, double speed) {
        // howMuch is in inches. A negative howMuch moves backward.

        // fetch motor positions
        lfPos = motorFrontLeft.getCurrentPosition();
        rfPos = motorFrontRight.getCurrentPosition();
        lrPos = motorBackLeft.getCurrentPosition();
        rrPos = motorBackRight.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * COUNTS_PER_INCH;
        rfPos -= howMuch * COUNTS_PER_INCH;
        lrPos -= howMuch * COUNTS_PER_INCH;
        rrPos += howMuch * COUNTS_PER_INCH;

        // move robot to new position
        motorFrontLeft.setTargetPosition(lfPos);
        motorFrontRight.setTargetPosition(rfPos);
        motorBackLeft.setTargetPosition(lrPos);
        motorBackRight.setTargetPosition(rrPos);

        startRunToPosition();

        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackLeft.setPower(speed);
        motorBackRight.setPower(speed);

        // wait for move to complete
        while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() && motorBackRight.isBusy()) {

            // Display it for the driver.
            mode.telemetry.addLine("Strafe Right");
            mode.telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            mode.telemetry.addData("Actual", "%7d :%7d", motorFrontLeft.getCurrentPosition(),
                    motorFrontRight.getCurrentPosition(), motorBackLeft.getCurrentPosition(),
                    motorBackRight.getCurrentPosition());
            mode.telemetry.update();
        }

        // Stop all motion;
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        stopRunToPosition();
    }

    public void turnClockwise(int whatAngle, double speed) {
        // whatAngle is in degrees. A negative whatAngle turns counterclockwise.

        // fetch motor positions
        lfPos = motorFrontLeft.getCurrentPosition();
        rfPos = motorFrontRight.getCurrentPosition();
        lrPos = motorBackLeft.getCurrentPosition();
        rrPos = motorBackRight.getCurrentPosition();

        // calculate new targets
        lfPos += whatAngle * COUNTS_PER_DEGREE;
        rfPos -= whatAngle * COUNTS_PER_DEGREE;
        lrPos += whatAngle * COUNTS_PER_DEGREE;
        rrPos -= whatAngle * COUNTS_PER_DEGREE;

        // move robot to new position
        motorFrontLeft.setTargetPosition(lfPos);
        motorFrontRight.setTargetPosition(rfPos);
        motorBackLeft.setTargetPosition(lrPos);
        motorBackRight.setTargetPosition(rrPos);

        startRunToPosition();

        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackLeft.setPower(speed);
        motorBackRight.setPower(speed);

        // wait for move to complete
        while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() && motorBackRight.isBusy()) {

            // Display it for the driver.
            mode.telemetry.addLine("Turn Clockwise");
            mode.telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            mode.telemetry.addData("Actual", "%7d :%7d", motorFrontLeft.getCurrentPosition(),
                    motorFrontRight.getCurrentPosition(), motorBackLeft.getCurrentPosition(),
                    motorBackRight.getCurrentPosition());
            mode.telemetry.update();
        }

        // Stop all motion;
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        stopRunToPosition();
    }

    public void moveToLine(int howMuch, double speed) {
        // howMuch is in inches. The robot will stop if the line is found before
        // this distance is reached. A negative howMuch moves left, positive moves right.

        // fetch motor positions
        lfPos = motorFrontLeft.getCurrentPosition();
        rfPos = motorFrontRight.getCurrentPosition();
        lrPos = motorBackLeft.getCurrentPosition();
        rrPos = motorBackRight.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * COUNTS_PER_INCH;
        rfPos -= howMuch * COUNTS_PER_INCH;
        lrPos -= howMuch * COUNTS_PER_INCH;
        rrPos += howMuch * COUNTS_PER_INCH;

        // move robot to new position
        motorFrontLeft.setTargetPosition(lfPos);
        motorFrontRight.setTargetPosition(rfPos);
        motorBackLeft.setTargetPosition(lrPos);
        motorBackRight.setTargetPosition(rrPos);

        startRunToPosition();

        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackLeft.setPower(speed);
        motorBackRight.setPower(speed);

        // wait for move to complete
        while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() && motorBackRight.isBusy()) {
            //if (mrOds.getLightDetected() > lineThreshold) break;

            // Display it for the driver.
            mode.telemetry.addLine("Move To Line");
            mode.telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            mode. telemetry.addData("Actual", "%7d :%7d", motorFrontLeft.getCurrentPosition(),
                    motorFrontRight.getCurrentPosition(), motorBackLeft.getCurrentPosition(),
                    motorBackRight.getCurrentPosition());
            mode.telemetry.update();
        }

        // Stop all motion;
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        stopRunToPosition();
    }
}

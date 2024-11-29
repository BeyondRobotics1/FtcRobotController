package org.firstinspires.ftc.teamcode;


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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.Helper;
import org.firstinspires.ftc.teamcode.common.PID;


public class SimpleDriveTrain {


    // The IMU sensor object
    IMU imu = null;

    //Use DcMotorEx to support bulk read.
    DcMotorEx motorFrontLeft;
    DcMotorEx motorBackLeft;
    DcMotorEx motorFrontRight;
    DcMotorEx motorBackRight;

    LinearOpMode mode;

    private final ElapsedTime runtime = new ElapsedTime();
    private int lfPos;
    private int rfPos;
    private int lrPos;
    private int rrPos;

    //for teleop
    //adjust forward/backward, left/right, and rotation power
    private double y_power_scale = 0.9; //forward/backward power adjustment
    private double x_power_scale = 0.9; //left/right power adjustment, make it slower
    private double rx_power_scale = 0.65;//rotation power adjustment, make it slower


    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     MOVE_FORWARD_ADJUSTMENT = 0.95; //adjustment moving forward auto
    static final double     MOVE_BACKWARD_ADJUSTMENT = 0.95; //adjustment moving backward auto
    static final double     MOVE_LEFT_ADJUSTMENT = 1.1; //adjustment moving backward auto

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: goBilda Motor Encoder
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
    public SimpleDriveTrain(HardwareMap hardwareMap, LinearOpMode mode, boolean hasIMU)
    {

        this.mode = mode;

        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "leftFront"); //hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "leftBack"); //hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "rightFront"); //hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "rightBack"); //hardwareMap.dcMotor.get("motorBackRight");

        //Reverse motors
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //distance sensor
//        distanceSensorSideLeft = hardwareMap.get(DistanceSensor.class, "dsLeftLeft");
//        distanceSensorSideRight = hardwareMap.get(DistanceSensor.class, "dsRightRight");
        //distanceSensorFrontLeft = hardwareMap.get(DistanceSensor.class, "dsRightForward");
        //distanceSensorFrontRight = hardwareMap.get(DistanceSensor.class, "dsLeftForward");

        if(hasIMU) {
            try {
                // Retrieve and initialize the IMU.
                imu = hardwareMap.get(IMU.class, "imu2");

                // The next two lines define Hub orientation.
                RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;//.RIGHT;
                RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;//.UP;

                RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

                // Now initialize the IMU with this mounting orientation
                // Note: if you choose two conflicting directions, this initialization will cause a code exception.
                imu.initialize(new IMU.Parameters(orientationOnRobot));
            } catch (Exception e)
            {
                imu = null;
            }
        }
    }

    /**
     * get the imu instance
     * @return
     */
    public IMU getImu() {
        return imu;
    }

    /**
     * reset the IMU, should be called by auto and NOT be called by teleop
     */
    public void resetYaw()
    {
        if(imu != null)
            imu.resetYaw();
    }

    /**
     * Stop motors and reset encoder STOP_AND_RESET_ENCODER,
     * then set motor to RUN_USING_ENCODER.
     * Called by auto
     */
    public void resetAndRunUsingEncoder()
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

        //field centric uses IMU, if no IMU, just do nothing

        //
        double y = left_stick_y;
        double x = left_stick_x;
        double rx = right_stick_x * rx_power_scale; //Helper.squareWithSign(right_stick_x);

        if(x != 0.0 && Math.abs(y/x) >= 1.2)
            x = 0.0;

        if(y != 0.0 && Math.abs(x/y) >= 1.2)
            y = 0.0;

        y *= y_power_scale;//Helper.squareWithSign(left_stick_y); // Remember, this is reversed!
        x *= x_power_scale * 1.1;//Helper.squareWithSign(left_stick_x * 1.1); // Counteract imperfect strafing

        //Read heading

        //Read inverse IMU heading, as the IMU heading is CW positive
        double rotX = Helper.squareWithSign(x * Math.cos(-botHeading) - y * Math.sin(-botHeading));
        double rotY = Helper.squareWithSign(x * Math.sin(-botHeading) + y * Math.cos(-botHeading));

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = (rotY  + rotX  + rx) / denominator;
        double backLeftPower = (rotY  - rotX  + rx) / denominator;
        double frontRightPower = (rotY  - rotX  - rx) / denominator;
        double backRightPower = (rotY  + rotX  - rx) / denominator;

        setMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        //mode.telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", botHeading * 180 / Math.PI);
    }

    // Distances in inches, angles in deg, speed 0.0 to 0.6
    // move forward
    public void moveLeft(double howMuch, double speed) {
        // howMuch is in inches. A negative howMuch moves backward.

        // fetch motor positions
        lfPos = motorFrontLeft.getCurrentPosition();
        rfPos = motorFrontRight.getCurrentPosition();
        lrPos = motorBackLeft.getCurrentPosition();
        rrPos = motorBackRight.getCurrentPosition();

        // calculate new targets
        lfPos -= howMuch * COUNTS_PER_INCH * MOVE_LEFT_ADJUSTMENT;
        rfPos += howMuch * COUNTS_PER_INCH * MOVE_LEFT_ADJUSTMENT;
        lrPos += howMuch * COUNTS_PER_INCH * MOVE_LEFT_ADJUSTMENT;
        rrPos -= howMuch * COUNTS_PER_INCH * MOVE_LEFT_ADJUSTMENT;

        // move robot to new position
        setMotorTargetPosition(lfPos, rfPos, lrPos, rrPos);
        setRunToPosition();
        setMotorPower(speed, speed, speed, speed);

        // wait for move to complete
        while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() && motorBackRight.isBusy()) {

//            // Display it for the driver.
//            mode.telemetry.addLine("Move left");
//            mode.telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
//            mode.telemetry.addData("Actual", "%7d :%7d", motorFrontLeft.getCurrentPosition(),
//                    motorFrontRight.getCurrentPosition(), motorBackLeft.getCurrentPosition(),
//                    motorBackRight.getCurrentPosition());
//            mode.telemetry.update();
        }

        // Stop all motion;
        setMotorPower(0, 0, 0, 0);

        setRunUsingEncoder();
    }

    /**
     *  // howMuch is in inches. A negative howMuch moves backward.
     * @param howMuch
     * @param speed
     */
    public void moveForward(double howMuch, double speed) {
        // fetch motor positions
        lfPos = motorFrontLeft.getCurrentPosition();
        rfPos = motorFrontRight.getCurrentPosition();
        lrPos = motorBackLeft.getCurrentPosition();
        rrPos = motorBackRight.getCurrentPosition();

        //get the adjustment for inches
        double adj = MOVE_FORWARD_ADJUSTMENT;
        if(howMuch < 0)
            adj = MOVE_BACKWARD_ADJUSTMENT;

        // calculate new targets
        lfPos += howMuch * COUNTS_PER_INCH * adj;
        rfPos += howMuch * COUNTS_PER_INCH * adj;
        lrPos += howMuch * COUNTS_PER_INCH * adj;
        rrPos += howMuch * COUNTS_PER_INCH * adj;

        // move robot to new position
        setMotorTargetPosition(lfPos, rfPos, lrPos, rrPos);
        setRunToPosition();
        setMotorPower(speed, speed, speed, speed);

        // wait for move to complete
        while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() && motorBackRight.isBusy()) {

            //// Display it for the driver.
//            mode.telemetry.addLine("Move forward");
//            mode.telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
//            mode.telemetry.addData("Actual", "%7d :%7d", motorFrontLeft.getCurrentPosition(),
//                    motorFrontRight.getCurrentPosition(), motorBackLeft.getCurrentPosition(),
//                    motorBackRight.getCurrentPosition());
//            mode.telemetry.update();
        }

        // Stop all motion;

        setMotorPower(0, 0, 0, 0);

        setRunUsingEncoder();
    }

    /**
     * Rumping up/down, based on our FLL code
     * @param howMuch: howMuch is in inches. A negative howMuch moves backward.
     * @param speedMin: minimum speed
     * @param speedMax: maximum speed
     * @param overRange: overrange for verocity profile, default to 1.25
     */
    public void moveForwardRamp(double howMuch, double speedMin, double speedMax, double overRange) {

        // fetch motor positions
        lfPos = motorFrontLeft.getCurrentPosition();
        rfPos = motorFrontRight.getCurrentPosition();
        lrPos = motorBackLeft.getCurrentPosition();
        rrPos = motorBackRight.getCurrentPosition();

        // remember the current position as start position
        int startPosition = lfPos;

        //get the adjustment for inches
        double adj = MOVE_FORWARD_ADJUSTMENT;
        if(howMuch < 0)
            adj = MOVE_BACKWARD_ADJUSTMENT;

        // calculate new targets
        lfPos += howMuch * COUNTS_PER_INCH * adj;
        rfPos += howMuch * COUNTS_PER_INCH * adj;
        lrPos += howMuch * COUNTS_PER_INCH * adj;
        rrPos += howMuch * COUNTS_PER_INCH * adj;

        int targetPosition = lfPos;
        double totalPositionChange = Math.abs(targetPosition - startPosition);

        // move robot to new position
        setMotorTargetPosition(lfPos, rfPos, lrPos, rrPos);
        setRunToPosition();

        double newSpeed = speedMin;
        double speedRange = (speedMax - speedMin) * overRange;//1.25;

        setMotorPower(newSpeed, newSpeed, newSpeed, newSpeed);

        //Log log = new Log("moveForwardRamp", true);

        // wait for move to complete
        while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() && motorBackRight.isBusy()) {

            //find out the percentage of distance traveled
            int currentPosition = motorFrontLeft.getCurrentPosition();
            double percentComplete = (Math.abs(currentPosition - startPosition) * 1.0) / totalPositionChange;

            //if(percentComplete > 0.6)
            //    percentComplete *= 0.9;

            //based on the percentage, find the new power value
            newSpeed = Math.min(speedMin + speedRange * Math.sin(percentComplete * Math.PI), speedMax);
            setMotorPower(newSpeed, newSpeed, newSpeed, newSpeed);

            //log.addData(startPosition);
            //log.addData(currentPosition);
            //log.addData(totalPositionChange);
            //log.addData(percentComplete);
            //log.addData(newSpeed);

            //log.update();
        }

        //log.close();

        //Stop all motion
        setMotorPower(0, 0, 0, 0);

        //Turn off RUN_TO_POSITION
        setRunUsingEncoder();
    }

    /**
     * Move forward with PID using IMU heading
     * We don't use this method now since the Gyro reading goes to 0 and stuck sometimes
     *
     * @param howMuch howMuch is in inches. A negative howMuch moves backward.
     * @param speed motor speed
     */
    public void moveForwardWithGyro(double howMuch, double speed) {
        //if there is no imu, do nothin
        if(imu == null)
            return;

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
        setMotorTargetPosition(lfPos, rfPos, lrPos, rrPos);
        setRunToPosition();
        setMotorPower(speed, speed, speed, speed);

        PID pid=new PID(0.05, 0, 0, 0);

        pid.setIntegrationBounds(-10, 10);

        // wait for move to complete
        while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() && motorBackRight.isBusy()) {

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double powerAdjustment = pid.calculate(botHeading);

            if(howMuch > 0) //forward
                setMotorPower(speed-powerAdjustment, speed+powerAdjustment, speed-powerAdjustment, speed+powerAdjustment);
            else //backward
                setMotorPower(speed+powerAdjustment, speed-powerAdjustment, speed+powerAdjustment, speed-powerAdjustment);

//            // Display it for the driver.
//            mode.telemetry.addLine("Move forward");
//            mode.telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
//            mode.telemetry.addData("Actual", "%7d :%7d", motorFrontLeft.getCurrentPosition(),
//                    motorFrontRight.getCurrentPosition(), motorBackLeft.getCurrentPosition(),
//                    motorBackRight.getCurrentPosition());
//
//            mode.telemetry.addData("Adjustment and Heading", "%7f :%7f", powerAdjustment, botHeading);
//
//            mode.telemetry.update();
        }

        // Stop all motion;

        setMotorPower(0, 0, 0, 0);

        setRunUsingEncoder();
    }

    /**
     * Turn robot to IMU's heading angle
     * @param targetHeadingDegree: turn to gyro target heading in degree,
     *                           positive counter clockwise, negative clockwise
     * @param speed: turn speed
     */
    public void turnToGyroHeading(int targetHeadingDegree, double speed)
    {
        //if there is no imu, use encoder turn
        if(imu == null) {
            turnClockwise(-targetHeadingDegree, speed);
            return;
        }


        //adjust the heading (0.92 default) as needed
        double targetHeading = targetHeadingDegree * 0.89;

        //current heading
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        //check if the target heading and current heading are closer enough
        //if yes, already reached the target, do nothing
        if(Math.abs(targetHeading - botHeading) <= 1)
            return;

        //IMU could stuck in 0, in this case, we use regular turn
        // fetch motor positions
        lfPos = motorFrontLeft.getCurrentPosition();
        rfPos = motorFrontRight.getCurrentPosition();
        lrPos = motorBackLeft.getCurrentPosition();
        rrPos = motorBackRight.getCurrentPosition();

        // calculate new targets (clock wise, the reverse direction of IMU)
        lfPos -= targetHeadingDegree * COUNTS_PER_DEGREE;
        rfPos += targetHeadingDegree * COUNTS_PER_DEGREE;
        lrPos -= targetHeadingDegree * COUNTS_PER_DEGREE;
        rrPos += targetHeadingDegree * COUNTS_PER_DEGREE;


        setRunUsingEncoder();

        if(targetHeading < botHeading) //turn  clock wise
        {
            setMotorPower(speed, -speed, speed, -speed);
        }
        else if ( targetHeading > botHeading) //turn counter-clock wise
        {
            setMotorPower(-speed, speed, -speed, speed);
        }

        int bad_yaw_count = 0;

        //keep turning until the difference of target heading and
        // the current heading yaw angle is no bigger than 1 degree
        while (Math.abs(targetHeading - botHeading) > 1) {    //original is 0.92, 0.9 works a bit better than 0.92
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            //is heading stuck at 0
            if(Math.abs(botHeading) <= 0.01)
                bad_yaw_count++;

            //IMU stuck at 0,
            if(bad_yaw_count >= 24)
                break;
        }

//        mode.telemetry.addData("Heading", botHeading);
//        mode.telemetry.update();

        //bad IMU, just turn targetHeading (12)
        if(bad_yaw_count >= 24)
        {
            setMotorTargetPosition(lfPos, rfPos, lrPos, rrPos);

            setRunToPosition();
            setMotorPower(speed, speed, speed, speed);

            // wait for move to complete
            while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() &&
                    motorBackLeft.isBusy() && motorBackRight.isBusy()) {

            }
            // Stop all motion;
            setMotorPower(0, 0, 0, 0);

            setRunUsingEncoder();

            mode.telemetry.addData("BAD IMU", bad_yaw_count);
            mode.telemetry.addData("Heading", botHeading);
            mode.telemetry.update();
        }
        else {
            setMotorPower(0, 0, 0, 0);

            mode.telemetry.addData("GOOD IMU", bad_yaw_count);
            mode.telemetry.addData("Heading", botHeading);
            mode.telemetry.update();
        }
    }


    /**
     * * Turn robot to IMU's heading angle
     * @param targetHeading: turn to gyro target heading in degree,
     *                           positive counter clockwise, negative clockwise
     * @param speed: turn speed
     * @param timeOut: timeout in milliseconds
     */
    public void fineTuneToGyroHeading(int targetHeading, double speed, int timeOut)
    {
        //if there is no imu, do nothing
        if(imu == null)
            return;

        //current heading
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        //check if the target heading and current heading are closer enough
        //if yes, already reached the target, do nothing
        if(Math.abs(targetHeading - botHeading) <= 1)
            return;

        //We use this timer to check the game time that has elapsed
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        if(targetHeading < botHeading) //turn  clock wise
        {
            setMotorPower(speed, -speed, speed, -speed);
        }
        else if ( targetHeading > botHeading) //turn counter-clock wise
        {
            setMotorPower(-speed, speed, -speed, speed);
        }

        //keep turning until the difference of target heading and
        // the current heading yaw angle is no bigger than 1 degree
        while (Math.abs(targetHeading - botHeading) > 1 && //original is 0.92, 0.9 works a bit better than 0.92
                timer.milliseconds() < timeOut) {
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        setMotorPower(0, 0, 0, 0);

        mode.telemetry.addData("Heading", botHeading);
        mode.telemetry.update();
    }

    /**
     * Turn robot clockwise
     * @param whatAngle: angle in degrees
     * @param speed: motor power
     */
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
        setMotorTargetPosition(lfPos, rfPos, lrPos, rrPos);
        setRunToPosition();
        setMotorPower(speed, speed, speed, speed);

        // wait for move to complete
        while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() && motorBackRight.isBusy()) {

//            // Display it for the driver.
//            mode.telemetry.addLine("Turn Clockwise");
//            mode.telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
//            mode.telemetry.addData("Actual", "%7d :%7d", motorFrontLeft.getCurrentPosition(),
//                    motorFrontRight.getCurrentPosition(), motorBackLeft.getCurrentPosition(),
//                    motorBackRight.getCurrentPosition());
//            mode.telemetry.update();
        }

        // Stop all motion;
        setMotorPower(0, 0, 0, 0);

        setRunUsingEncoder();
    }

    public void moveToLine(double howMuch, double speed) {
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
        setMotorTargetPosition(lfPos, rfPos, lrPos, rrPos);
        setRunToPosition();
        setMotorPower(speed, speed, speed, speed);

        // wait for move to complete
        while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() && motorBackRight.isBusy()) {
            //if (mrOds.getLightDetected() > lineThreshold) break;

//            // Display it for the driver.
//            mode.telemetry.addLine("Move To Line");
//            mode.telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
//            mode.telemetry.addData("Actual", "%7d :%7d", motorFrontLeft.getCurrentPosition(),
//                    motorFrontRight.getCurrentPosition(), motorBackLeft.getCurrentPosition(),
//                    motorBackRight.getCurrentPosition());
//            mode.telemetry.update();
        }

        // Stop all motion;
        setMotorPower(0, 0, 0, 0);

        setRunUsingEncoder();
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

    //set power for each motors
    public void setMotorTargetPosition(int frontLeft, int frontRight,
                                       int backLeft, int backRight)
    {
        motorFrontLeft.setTargetPosition(frontLeft);
        motorFrontRight.setTargetPosition(frontRight);
        motorBackLeft.setTargetPosition(backLeft);
        motorBackRight.setTargetPosition(backRight);
    }

    //set all four motors to RUN_TO_POSITION
    public void setRunToPosition()
    {
        //if(DcMotor.RunMode.RUN_TO_POSITION != motorFrontLeft.getMode()) {
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //}
    }

    //set all four motors to RUN_USING_ENCODER
    public void setRunUsingEncoder()
    {
        //if(DcMotor.RunMode.RUN_USING_ENCODER != motorFrontLeft.getMode()) {
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //}
    }

    //set all four motors to RUN_WITHOUT_ENCODER
    public void setRunWithoutEncoder()
    {
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void changeXYPowerScale(double delta)
    {
        y_power_scale += delta;
        x_power_scale += delta;

        y_power_scale = Range.clip(y_power_scale, 0, 1);
        x_power_scale = Range.clip(x_power_scale, 0, 1);

        //mode.telemetry.addData("Y Power Scale", "%.2f", y_power_scale);
        //mode.telemetry.addData("X Power Scale", "%.2f", x_power_scale);
    }

    public void changeRXPowerScale(double delta)
    {
        rx_power_scale += delta;
        rx_power_scale = Range.clip(rx_power_scale, 0, 1);

        //mode.telemetry.addData("RX Power Scale", "%.2f", rx_power_scale);
    }
}

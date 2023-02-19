package org.firstinspires.ftc.teamcode;


import android.icu.util.TimeUnit;

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


public class DriveTrain {

    //REV 2m distance sensors
    DistanceSensor distanceSensorFrontLeft;
    DistanceSensor distanceSensorFrontRight;
    DistanceSensor distanceSensorSideLeft;
    DistanceSensor distanceSensorSideRight;

    // The IMU sensor object
    IMU imu = null;

    //Use DcMotorEx to support bulk read.
    DcMotorEx motorFrontLeft;
    DcMotorEx motorBackLeft;
    DcMotorEx motorFrontRight;
    DcMotorEx motorBackRight;

    LinearOpMode mode;

    private ElapsedTime runtime = new ElapsedTime();
    private int lfPos;
    private int rfPos;
    private int lrPos;
    private int rrPos;
    private double lineThreshold = 0.7; // floor should be below this value, line above
    private double redThreshold = 1.9; // red should be below this value, blue above


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
    public DriveTrain(HardwareMap hardwareMap, LinearOpMode mode, boolean hasIMU)
    {

        this.mode = mode;

        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft"); //hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft"); //hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight"); //hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight"); //hardwareMap.dcMotor.get("motorBackRight");

        //Reverse motors
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //distance sensor
        distanceSensorSideLeft = hardwareMap.get(DistanceSensor.class, "dsLeftLeft");
        distanceSensorSideRight = hardwareMap.get(DistanceSensor.class, "dsRightRight");
        distanceSensorFrontLeft = hardwareMap.get(DistanceSensor.class, "dsLeftForward");
        distanceSensorFrontRight = hardwareMap.get(DistanceSensor.class, "dsRightForward");

        if(hasIMU) {
            try {
                // Retrieve and initialize the IMU.
                imu = hardwareMap.get(IMU.class, "imu");

                // The next two lines define Hub orientation.
                RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
                RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

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
                         double right_stick_x) {

        //field centric uses IMU, if no IMU, just do nothing
        if(imu == null)
            return;

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
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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
        //if there is no imu, do nothing
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
     * @param targetHeading: turn to gyro target heading, positive counter clock wise, negative clock
     * @param speed: turn speed
     */
    public void turnToGyroHeading(int targetHeading, double speed)
    {
        //if there is no imu, use encoder turn
        if(imu == null) {
            turnClockwise(-targetHeading, speed);
            return;
        }

        //current heading
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if(botHeading == targetHeading)
            return;

        // fetch motor positions
        lfPos = motorFrontLeft.getCurrentPosition();
        rfPos = motorFrontRight.getCurrentPosition();
        lrPos = motorBackLeft.getCurrentPosition();
        rrPos = motorBackRight.getCurrentPosition();

        // calculate new targets (clock wise, the reverse direction of IMU)
        lfPos -= targetHeading * COUNTS_PER_DEGREE;
        rfPos += targetHeading * COUNTS_PER_DEGREE;
        lrPos -= targetHeading * COUNTS_PER_DEGREE;
        rrPos += targetHeading * COUNTS_PER_DEGREE;


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

        //adjust the adjust (0.92 default) as needed
        while (Math.abs(targetHeading * 0.92 - botHeading) > 1) {
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            if(Math.abs(botHeading) <= 0.01)
                bad_yaw_count++;

            //IMU stuck at 0
            if(bad_yaw_count >= 12)
                break;
        }

//        mode.telemetry.addData("Heading", botHeading);
//        mode.telemetry.update();

        //bad IMU, just turn targetHeading
        if(bad_yaw_count >= 12)
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

    /**
     * move forward or backward to the #th junction pole
     * @param left: true use the left distance sensor, false, use the right one
     * @param poleToCount: the number of poles to reach
     * @param speed: motor power, negative will move backward
     * @return
     */
    public double moveToPole(boolean left, int poleToCount, double speed){

        double distanceToPole = 0;

        //create a pole detector instance
        PoleDetector detector;

        if(left)
            detector = new PoleDetector(distanceSensorSideLeft);
        else
            detector = new PoleDetector(distanceSensorSideRight);

        setRunUsingEncoder();
        setMotorPower(speed, speed, speed, speed);

        //Log log = new Log("pole_detection_power_0.6", true);

        while(true) {
            //find out how many poles have been passed & detected
            int polesDetected = detector.detectPoles();
            distanceToPole = detector.getPoleDistance();

            //mode.telemetry.addData("current distance", detector.getCurrentDistance());
            //mode.telemetry.addData("Pole counted", polesDetected);
            //mode.telemetry.update();

            //log.addData(detector.getCurrentDistance());
            //log.update();

            if (polesDetected >= poleToCount)
                break;
        }

        //log.close();

        //stop all motors
        setMotorPower(0, 0, 0, 0);

        return distanceToPole;
    }

    /**
     * Square against two junction poles
     * @param poleToCount: the nth pole to square
     * @param speed: motor power, negative move backward
     * @param timeOut: time difference between left and right distance detection in milliseconds
     * @return the distance to left pole or right pole based on distance sensor
     */
    public SquareToPoolResult squareToPoles(int poleToCount, double speed, int timeOut){

        SquareToPoolResult result = new SquareToPoolResult();

        double distanceToPoleLeft = 3;
        double distanceToPoleRight = 3;

        //create a pole detector instance
        PoleDetector detectorLeft = new PoleDetector(distanceSensorSideLeft);
        PoleDetector detectorRight = new PoleDetector(distanceSensorSideRight);

        setRunUsingEncoder();
        setMotorPower(speed, speed, speed, speed);

        boolean leftDetect = false;
        boolean rightDetect = false;
        double leftPower = speed;
        double rightPower = speed;

        //Log log = new Log("pole_squaring"+(int)(Math.random()*1000), true);

        //We use this timer to check the game time that has elapsed
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while(true) {
            //find out how many poles have been passed & detected
            int polesDetectedLeft = detectorLeft.detectPoles();
            int polesDetectedRight = detectorRight.detectPoles();

            distanceToPoleLeft = detectorLeft.getPoleDistance();
            distanceToPoleRight = detectorRight.getPoleDistance();

            if (polesDetectedLeft >= poleToCount && !leftDetect) {
                leftDetect = true;

                leftPower = 0;

                setMotorPower(0, rightPower, 0, rightPower);

                timer.reset();
            }

            if (polesDetectedRight >= poleToCount && !rightDetect) {
                rightDetect = true;

                setMotorPower(leftPower, 0, leftPower, 0);

                rightPower = 0;

                timer.reset();
            }

            //log.addData(distanceToPoleLeft);
            //log.addData(distanceToPoleRight);
            //log.addData(leftDetect);
            //log.addData(rightDetect);
            //log.addData(timer.milliseconds());

            if (rightDetect && leftDetect) {
                result.left = true;
                result.distance = distanceToPoleLeft;

                //log.update();
                break;
             }

            if(leftDetect)
            {
                result.left = true;
                result.distance = distanceToPoleLeft;

                if(timer.milliseconds() >= timeOut) {
                    //log.update();
                    break;
                }
                //log.update();
            }
            else if (rightDetect)
            {
                result.left = false;
                result.distance = distanceToPoleRight;

                if(timer.milliseconds() >= timeOut) {
                    //log.update();
                    break;
                }
                //log.update();
            }
            //else
            //    log.update();
        }

        //log.close();

        //stop all motors
        setMotorPower(0, 0, 0, 0);

        return result;
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
        //if(DcMotor.RunMode.RUN_WITHOUT_ENCODER != motorFrontLeft.getMode()) {
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //}
    }


    //Left distance sensor
    public double getLeftDistanceINCH(){
        return distanceSensorSideLeft.getDistance(DistanceUnit.INCH);
    }

    //Left distance sensor
    public double getRightDistanceINCH(){
        return distanceSensorSideRight.getDistance(DistanceUnit.INCH);
    }

    //Front Left distance sensor
    public double getFrontLeftDistanceINCH(){
        return distanceSensorFrontLeft.getDistance(DistanceUnit.INCH);
    }

    //Front right distance sensor
    public double getFrontRightDistanceINCH(){
        return distanceSensorFrontRight.getDistance(DistanceUnit.INCH);
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

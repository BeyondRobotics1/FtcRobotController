package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.decode.OpMode.DecodeBlackBoard;
import org.opencv.core.Mat;

import java.util.List;

public class IMUTurret {
    //hardware
    LinearOpMode mode;
    private Servo turretLeft, turretRight;
    GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    //AnalogInput analogInput;
    Pose2D targetPose, startPose;
    double servoPositionRight = 0; //90 degree
    double servoPositionLeft = 0.42; //-90 degree
    double servoPositionMiddle = 0.205;//0 degree

    double robotDistanceToGoal = 0.;
    int alliance = DecodeBlackBoard.BLUE;

    //limelight is only need for auto
    //no limelight for teleop
    public IMUTurret(HardwareMap hardwareMap, LinearOpMode mode,
                     Pose2D robotPose, Pose2D targetPose,
                     int alliance, //1 or 2
                     boolean useLimeLight) {

        this.alliance = alliance;

        this.mode = mode;
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        //analogInput = hardwareMap.get(AnalogInput.class, "turretAnalogLeft");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure the sensor
        configurePinpoint();

        // Set the location of the robot - this should be the place you are starting the robot from
        //pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        this.startPose = new Pose2D(DistanceUnit.INCH,
                robotPose.getX(DistanceUnit.INCH),
                robotPose.getY(DistanceUnit.INCH),
                AngleUnit.DEGREES,
                robotPose.getHeading(AngleUnit.DEGREES));
        pinpoint.setPosition(robotPose);


        this.targetPose = new Pose2D(DistanceUnit.INCH,
                targetPose.getX(DistanceUnit.INCH),
                targetPose.getY(DistanceUnit.INCH),
                AngleUnit.DEGREES,
                targetPose.getHeading(AngleUnit.DEGREES));


        //use limelight
        if(useLimeLight) {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
        }


        setServoPosition(servoPositionMiddle);
    }

    public boolean isLimeLight3ARunning()
    {
        if(limelight != null)
            return limelight.isRunning();
        else
            return false;
    }

    public void setServoPosition(double power)
    {
        turretLeft.setPosition(power);
    }

    public double getServoPosition()
    {
        return turretLeft.getPosition();
    }

    public void setIMUPoseToRobotStartPose()
    {
        if(pinpoint != null)
            pinpoint.setPosition(startPose);
    }

    //set the turret to center heading
    //could be useful when auto aiming is not
    //working as expected
    public void resetTurretHeading()
    {
        setServoPosition(servoPositionMiddle);
    }

    //drive robot to align with the white tap
    //of the human player loading zone
    public void resetIMUPose()
    {
        if(alliance == DecodeBlackBoard.RED)
            setIMUPose(DecodeBlackBoard.RED_RESET_POSE);
        else
            setIMUPose(DecodeBlackBoard.BLUE_RESET_POSE);

    }

    public void setIMUPose(Pose2D robotPose)
    {
        if(pinpoint != null)
            pinpoint.setPosition(robotPose);
    }


    //return the distance from robot to goal
    //could use it to adjust shooter speed
    public double distanceToGoal()
    {
        return robotDistanceToGoal;
    }

    //use limelight to detect the April Tag of the Obelisk
    //return:
    //  0 - Obelisk April Tag is not detected
    //  21, 22, 23 - Obelisk April Tag is detected
    public int detectObeliskTagID()
    {
        int tagID = 0;

        if(limelight != null) {
            LLResult result = limelight.getLatestResult();

            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {

                int tempID = fr.getFiducialId();

                if (tempID >= 21 && tempID <= 23) {
                    tagID = tempID;
                    break;
                }
            }
        }

        return tagID;
    }

//    public double getAnalogAngleDegree()
//    {
//        return 0;
//
//        // get the voltage of our analog line
//        // divide by 3.3 (the max voltage) to get a value between 0 and 1
//        // multiply by 360 to convert it to 0 to 360 degrees
//        // double position = analogInput.getVoltage() / 3.3 * 360;
//       //return analogInput.getVoltage() / 3.3 * 360 - analogStartingAngle;
//    }

    public void autoAim()
    {

        // read from odometry pinpoint
        pinpoint.update();

        Pose2D pose2D = pinpoint.getPosition();

        double x = pose2D.getX(DistanceUnit.INCH);
        double y = pose2D.getY(DistanceUnit.INCH);
        double heading = pose2D.getHeading(AngleUnit.DEGREES);

        if(alliance == DecodeBlackBoard.RED)
            heading = -heading;

        //unwrap into [0 360]
        if(heading < 0)
            heading += 360;

        // target position
        double x0 = targetPose.getX(DistanceUnit.INCH);
        double y0 = targetPose.getY(DistanceUnit.INCH);

        //calculate distance
        robotDistanceToGoal = Math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0));


        // calculate alpha
        double alpha = Math.toDegrees(Math.atan2(Math.abs(x-x0), Math.abs(y-y0)));
        double theta;

        if(alliance == DecodeBlackBoard.RED)
            theta = -90 - alpha + heading;
        else
            theta = 90 + alpha - heading;

        // calculate servo position
        double servoPosition;

        if(alliance == DecodeBlackBoard.RED)
            servoPosition = servoPositionLeft * (theta + 90.0)/180.0;
        else
            servoPosition = servoPositionLeft * (theta + 90.0)/180.0;

        mode.telemetry.addData("Servo position nc:", servoPosition);


        if(servoPosition > servoPositionLeft){
            servoPosition = servoPositionLeft;
        }else if(servoPosition < servoPositionRight){
            servoPosition = servoPositionRight;
        }

        setServoPosition(servoPosition);

        mode.telemetry.addData("Servo position:", servoPosition);

        mode.telemetry.addData("IMU Heading:", heading);
        mode.telemetry.addData("IMU x:", x);
        mode.telemetry.addData("IMU y:", y);

        mode.telemetry.addData("Delta x:", Math.abs(x-x0));
        mode.telemetry.addData("Delta y:", Math.abs(y-y0));
        mode.telemetry.addData("alpha:", alpha);
        mode.telemetry.addData("theta:", theta);

    }

    //read the pinpoint readings
    public Pose2D readPinpoint()
    {
        // read from odometry pinpoint
        pinpoint.update();

        return pinpoint.getPosition();
    }


    public void configurePinpoint(){
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(-106, -143.6, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }
}

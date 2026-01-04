package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.decode.OpMode.DecodeBlackBoard;

/**
 * Using pinpoint for robot location in the field
 */
public class IMULocalizer {

    public enum RobotZone
    {
        NOT_IN_SHOOTING_ZONE,
        NEAR_SHOOTING_ZONE,
        MEDIUM_SHOOTING_ZONE,
        FAR_SHOOTING_ZONE,
        OUT_SHOOTING_ZONE
    }

    //know locations
    //red near zone
    public static final double RED_X1 = 24;
    public static final double RED_Y1 = 30;
    public static final double RED_X2 = 78;
    public static final double RED_Y2 = 90;
    public static final double RED_X3 = 128;
    public static final double RED_Y3 = 32;
    //red out zone
    public static final double RED_X4 = 56;
    public static final double RED_Y4 = 110;
    public static final double RED_X5 = 102;
    public static final double RED_Y5 = 110;

    //red near zone
    public static final double BLUE_X1 = 14;
    public static final double BLUE_Y1 = 114;
    public static final double BLUE_X2 = 62;
    public static final double BLUE_Y2 = 66;
    public static final double BLUE_X3 = 130;
    public static final double BLUE_Y3 = 114;
    //red out zone
    public static final double BLUE_X4 = 54;
    public static final double BLUE_Y4 = 30;
    public static final double BLUE_X5 = 88;
    public static final double BLUE_Y5 = 30;

    public static final double RED_FAR_ZONE_DISTANCE = 86;
    public static final double RED_MEDIUM_ZONE_DISTANCE = 70;

    public static final double BLUE_FAR_ZONE_DISTANCE = 70;
    public static final double BLUE_MEDIUM_ZONE_DISTANCE = 50;

    //hardware
    LinearOpMode mode;

    GoBildaPinpointDriver pinpoint;

    // target x, y coordinates in INCH
    double x0, y0;

    Pose2D startPose;

    double robotDistanceToGoal = 0.;
    int alliance = DecodeBlackBoard.BLUE;

    double botHeadingDegrees;
    double x, y; //coordinates in INCH
    boolean isHeadingToGoal = false;
    RobotZone robotZone = RobotZone.NOT_IN_SHOOTING_ZONE;

    public IMULocalizer(HardwareMap hardwareMap, LinearOpMode mode,
                        Pose2D robotPose, Pose2D targetPose,
                        int alliance)
    {
        this.alliance = alliance;
        this.mode = mode;

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

        //target coordinates
        x0 = targetPose.getX(DistanceUnit.INCH);
        y0 = targetPose.getY(DistanceUnit.INCH);
    }

    public double getHeading()
    {
        return botHeadingDegrees;
    }

    public double getX()
    {
        return x;
    }

    public double getY()
    {
        return y;
    }

    public double getRobotDistanceToGoal()
    {
        return  robotDistanceToGoal;
    }

    public boolean isHeadingToGoal() {
        return isHeadingToGoal;
    }

    public RobotZone getRobotZone() {
        return robotZone;
    }

    public void setIMUPoseToRobotStartPose()
    {
        if(pinpoint != null)
            pinpoint.setPosition(startPose);
    }

    public void setIMUPose(Pose2D robotPose)
    {
        if(pinpoint != null)
            pinpoint.setPosition(robotPose);
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

    public boolean update()
    {
        if(pinpoint == null)
            return false;

        // read from odometry pinpoint
        pinpoint.update();

        Pose2D pose2D = pinpoint.getPosition();

        x = pose2D.getX(DistanceUnit.INCH);
        y = pose2D.getY(DistanceUnit.INCH);
        botHeadingDegrees = pose2D.getHeading(AngleUnit.DEGREES);

        //calculate distance for auto shooting speed adjustment
        robotDistanceToGoal = Math.sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0));

        //find if bot is heading to the goal or not
        if (alliance == DecodeBlackBoard.RED)
        {
            detectRedRobotZone(x, y);

            if(botHeadingDegrees > -30 && botHeadingDegrees < 120)
                isHeadingToGoal = false;
            else
                isHeadingToGoal = true;
        }
        else
        {
            detectBlueRobotZone(x, y);

            if(botHeadingDegrees > -120 && botHeadingDegrees < 30)
                isHeadingToGoal = false;
            else
                isHeadingToGoal = true;
        }

        //
        return true;
    }

    private void detectBlueRobotZone(double x, double y)
    {
        //first check if it is in the out zone
        if(y <= BLUE_Y4 && (x >= BLUE_X4 && x <= BLUE_X5))
            robotZone = RobotZone.OUT_SHOOTING_ZONE;
        else if( y < BLUE_Y2 && y > BLUE_Y4)
            robotZone = RobotZone.NOT_IN_SHOOTING_ZONE;
        else
        {
            boolean inZone = false;

            if(x < BLUE_X2) {
                //check line 1 (x1, y1) - (x2, y2)
                double yint1 = (BLUE_Y1 * (BLUE_X2 - x) + BLUE_Y2 * (x - BLUE_X1)) / (BLUE_X2 - BLUE_X1);

                //mode.telemetry.addData("yint1:", yint1);

                if(y >= yint1)
                    inZone = true;
            }
            else {

                //check line 2 (x2, y2) - (x3, y3)
                double yint2 = (BLUE_Y2 * (BLUE_X3 - x) + BLUE_Y3 * (x - BLUE_X2)) / (BLUE_X3 - BLUE_X2);

                //mode.telemetry.addData("yint2:", yint2);

                if (y >= yint2)
                    inZone = true;
            }

            //in shooting zone
            if(inZone)
            {
                if(robotDistanceToGoal >= BLUE_FAR_ZONE_DISTANCE)
                    robotZone = RobotZone.FAR_SHOOTING_ZONE;
                else if (robotDistanceToGoal >= BLUE_MEDIUM_ZONE_DISTANCE)
                    robotZone = RobotZone.MEDIUM_SHOOTING_ZONE;
                else
                    robotZone = RobotZone.NEAR_SHOOTING_ZONE;
            }
            else {
                robotZone = RobotZone.NOT_IN_SHOOTING_ZONE;
            }
        }
    }


    private void detectRedRobotZone(double x, double y)
    {
        //first check if it is in the out zone
        if(y >= RED_Y4 && (x >= RED_X4 && x <= RED_X5))
            robotZone = RobotZone.OUT_SHOOTING_ZONE;
        else if( y > RED_Y2 && y < RED_Y4)
            robotZone = RobotZone.NOT_IN_SHOOTING_ZONE;
        else
        {
            boolean inZone = false;

            if(x < RED_X2) {
                //check line 1 (x1, y1) - (x2, y2)
                double yint1 = (RED_Y1 * (RED_X2 - x) + RED_Y2 * (x - RED_X1)) / (RED_X2 - RED_X1);

                mode.telemetry.addData("yint1:", yint1);

                if(y <= yint1)
                    inZone = true;
            }
            else {

                //check line 2 (x2, y2) - (x3, y3)
                double yint2 = (RED_Y2 * (RED_X3 - x) + RED_Y3 * (x - RED_X2)) / (RED_X3 - RED_X2);

                mode.telemetry.addData("yint2:", yint2);

                if (y <= yint2)
                    inZone = true;
            }

            //in shooting zone
            if(inZone)
            {
                if(robotDistanceToGoal >= RED_FAR_ZONE_DISTANCE)
                    robotZone = RobotZone.FAR_SHOOTING_ZONE;
                else if (robotDistanceToGoal >= RED_MEDIUM_ZONE_DISTANCE)
                    robotZone = RobotZone.MEDIUM_SHOOTING_ZONE;
                else
                    robotZone = RobotZone.NEAR_SHOOTING_ZONE;
            }
            else {
                  robotZone = RobotZone.NOT_IN_SHOOTING_ZONE;
            }
        }
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

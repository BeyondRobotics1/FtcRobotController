package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.biobuzz.opmode.BioBuzzDashboard;

public class IMULocalizer {

    public enum RobotZone
    {
        NOT_IN_SHOOTING_ZONE,
        NEAR_SHOOTING_ZONE,
        MEDIUM_SHOOTING_ZONE,
        FAR_SHOOTING_ZONE,
        FAR_FAR_SHOOTING_ZONE,
    }

    public static final double kRedYLimitRight = 32;//
    public static final double kRedYLimitLeft = BioBuzzDashboard.kFieldSize - kRedYLimitRight;

    public static final double kBlueYLimitRight = kRedYLimitLeft;//
    public static final double kBlueYLimitLeft = kRedYLimitRight;//


    public static final double kFAR_FAR_ZONE_DISTANCE = 82;//80;
    public static final double kFAR_ZONE_DISTANCE = 66;//80;
    public static final double kMEDIUM_ZONE_DISTANCE = 52;//44;


    LinearOpMode mode;
    GoBildaPinpointDriver pinpoint;

    // target x, y coordinates in INCH
    double x0, y0;

    Pose2D startPose;
    Pose2D targetPose;

    double robotDistanceToGoal = 0.;
    int alliance = BioBuzzDashboard.kAllianceBlue;

    double botHeadingDegrees;
    double x, y; //coordinates in INCH
    boolean isHeadingToGoal = true; //is robot heading to the hiv
    boolean isAudienceSide = true; //is robot is audience side
    IMULocalizer.RobotZone robotZone = IMULocalizer.RobotZone.NOT_IN_SHOOTING_ZONE;

    public IMULocalizer(HardwareMap hardwareMap, LinearOpMode mode,
                        Pose2D robotPose,
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

        updateTargetPose(robotPose.getY(DistanceUnit.INCH));
    }

    public double getHeading()
    {
        return botHeadingDegrees;
    }

    //return robot's current X, Y position
    public double getX()
    {
        return x;
    }

    public double getY()
    {
        return y;
    }

    //return the hiv target X, Y position
    //the target position changes according to robot's position
    public double getX0()
    {
        return targetPose.getX(DistanceUnit.INCH);
    }

    public double getY0()
    {
        return targetPose.getY(DistanceUnit.INCH);
    }

    public double getRobotDistanceToGoal()
    {
        return  robotDistanceToGoal;
    }

    public boolean isHeadingToGoal() {
        return isHeadingToGoal;
    }

    //check if robot is in audience side
    public boolean isAudienceSide()
    {
        return isAudienceSide;
    }

    public IMULocalizer.RobotZone getRobotZone() {
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

        updateTargetPose(y);
        x0 = targetPose.getX(DistanceUnit.INCH);
        y0 = targetPose.getY(DistanceUnit.INCH);

        //calculate distance for auto shooting speed adjustment
        robotDistanceToGoal = Math.sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0));

        //find if bot is heading to the goal or not
        if (alliance == BioBuzzDashboard.kAllianceRed)
        {
            detectRobotZone(x, y, robotDistanceToGoal);

            isHeadingToGoal = true;//!(botHeadingDegrees > 0) || !(botHeadingDegrees < 80);
        }
        else
        {
            detectRobotZone(x, y, robotDistanceToGoal);

            isHeadingToGoal = true;//!(botHeadingDegrees > -80) || !(botHeadingDegrees < 0);
        }

        //
        return true;
    }

    private void updateTargetPose(double y)
    {
        if(alliance == BioBuzzDashboard.kAllianceRed)
        {
            //audience side target
            if(y <= 70.75)
                targetPose = BioBuzzDashboard.kRedTargetPoseAudience;
            else //score side target
                targetPose = BioBuzzDashboard.kRedTargetPoseScore;
        }
        else {
            if(y <= 70.5)
                targetPose = BioBuzzDashboard.kBlueTargetPoseScore;
            else
                targetPose = BioBuzzDashboard.kBlueTargetPoseAudience;
        }
    }

    private void detectRobotZone(double x, double y, double distanceToHive)
    {
        //TBD
        if(alliance == BioBuzzDashboard.kAllianceRed)
        {
            if(y <= kRedYLimitLeft) {
                isAudienceSide = true;
                robotZone = RobotZone.MEDIUM_SHOOTING_ZONE;
            }
            else if(y >= kRedYLimitRight) {
                isAudienceSide = false;
                robotZone = RobotZone.MEDIUM_SHOOTING_ZONE;
            }
            else
                robotZone = RobotZone.NOT_IN_SHOOTING_ZONE;
        }
        else
        {
            if(y <= kRedYLimitLeft) {
                isAudienceSide = false;
                robotZone = RobotZone.MEDIUM_SHOOTING_ZONE;
            }
            else if(y >= kRedYLimitRight) {
                isAudienceSide = true;
                robotZone = RobotZone.MEDIUM_SHOOTING_ZONE;
            }
            else
                robotZone = RobotZone.NOT_IN_SHOOTING_ZONE;
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
        //118, -143.6
        pinpoint.setOffsets(96, -143.72, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

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

package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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
        FAR_FAR_SHOOTING_ZONE,
        OUT_SHOOTING_ZONE
    }

    //know locations
    //red near zone
    public static final double RED_X1 = 20;//22;//24
    public static final double RED_Y1 = 30;//33; //30
    public static final double RED_X2 = 70;//70;
    public static final double RED_Y2 = 84;//84;
    public static final double RED_X3 = 121;//122;//128
    public static final double RED_Y3 = 30;//32; //32
    //red out zone
    public static final double RED_X4 = 52;
    public static final double RED_Y4 = 110;
    public static final double RED_X5 = 84;
    public static final double RED_Y5 = 110;

    //red near zone
    public static final double BLUE_X1 = 20;// 23;
    public static final double BLUE_Y1 = 111;//107;
    public static final double BLUE_X2 = 70;//70;
    public static final double BLUE_Y2 = 57;//57;
    public static final double BLUE_X3 = 121;//122;
    public static final double BLUE_Y3 = 109;//105;
    //red out zone
    public static final double BLUE_X4 = 52;
    public static final double BLUE_Y4 = 32;//39; //36
    public static final double BLUE_X5 = 84;
    public static final double BLUE_Y5 = 32;//39;

    public static final double RED_FAR_FAR_ZONE_DISTANCE = 82;//80;
    public static final double RED_FAR_ZONE_DISTANCE = 66;//80;
    public static final double RED_MEDIUM_ZONE_DISTANCE = 52;//44;

    public static final double BLUE_FAR_FAR_ZONE_DISTANCE = 82;//80;
    public static final double BLUE_FAR_ZONE_DISTANCE = 66;//80;
    public static final double BLUE_MEDIUM_ZONE_DISTANCE = 52;//44;

    //hardware
    LinearOpMode mode;

    //GoBildaPinpointDriver pinpoint;
    private Follower follower;

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
                        Follower follower,
                        Pose2D robotPose, Pose2D targetPose,
                        int alliance)
    {
        this.alliance = alliance;
        this.mode = mode;
        this.follower = follower;

//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//
//        // Configure the sensor
//        configurePinpoint();

        // Set the location of the robot - this should be the place you are starting the robot from
        //pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        this.startPose = new Pose2D(DistanceUnit.INCH,
                robotPose.getX(DistanceUnit.INCH),
                robotPose.getY(DistanceUnit.INCH),
                AngleUnit.DEGREES,
                robotPose.getHeading(AngleUnit.DEGREES));

        setIMUPoseToRobotStartPose();

        //pinpoint.setPosition(robotPose);

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
        if(follower != null) {
            follower.setStartingPose(new Pose(startPose.getX(DistanceUnit.INCH),
                    startPose.getY(DistanceUnit.INCH), startPose.getHeading(AngleUnit.RADIANS)));
            follower.update();
        }
    }

    public void setIMUPose(Pose2D robotPose)
    {
        if(follower != null) {
            follower.setPose(new Pose(robotPose.getX(DistanceUnit.INCH),
                    robotPose.getY(DistanceUnit.INCH), robotPose.getHeading(AngleUnit.RADIANS)));
            follower.update();
        }
    }

    public boolean update()
    {
        if(follower == null)
            return false;

        // read from odometry pinpoint
        follower.update();

        Pose p = follower.getPose();

        x = p.getX();
        y = p.getY();
        botHeadingDegrees = Math.toDegrees(p.getHeading());

        //calculate distance for auto shooting speed adjustment
        robotDistanceToGoal = Math.sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0));

        //find if bot is heading to the goal or not
        if (alliance == DecodeBlackBoard.RED)
        {
            detectRedRobotZone(x, y);

            //if(botHeadingDegrees > -30 && botHeadingDegrees < 120)
            if(botHeadingDegrees > 0 && botHeadingDegrees < 80)
                isHeadingToGoal = false;
            else
                isHeadingToGoal = true;
        }
        else
        {
            detectBlueRobotZone(x, y);

            //if(botHeadingDegrees > -120 && botHeadingDegrees < 30)
            if(botHeadingDegrees > -80 && botHeadingDegrees < 0)
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
                if(robotDistanceToGoal >= BLUE_FAR_FAR_ZONE_DISTANCE)
                    robotZone = RobotZone.FAR_FAR_SHOOTING_ZONE;
                else if(robotDistanceToGoal >= BLUE_FAR_ZONE_DISTANCE)
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

                //mode.telemetry.addData("yint1:", yint1);

                if(y <= yint1)
                    inZone = true;
            }
            else {

                //check line 2 (x2, y2) - (x3, y3)
                double yint2 = (RED_Y2 * (RED_X3 - x) + RED_Y3 * (x - RED_X2)) / (RED_X3 - RED_X2);

                //mode.telemetry.addData("yint2:", yint2);

                if (y <= yint2)
                    inZone = true;
            }

            //in shooting zone
            if(inZone)
            {
                if(robotDistanceToGoal >= RED_FAR_FAR_ZONE_DISTANCE)
                    robotZone = RobotZone.FAR_FAR_SHOOTING_ZONE;
                else if(robotDistanceToGoal >= RED_FAR_ZONE_DISTANCE)
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
}

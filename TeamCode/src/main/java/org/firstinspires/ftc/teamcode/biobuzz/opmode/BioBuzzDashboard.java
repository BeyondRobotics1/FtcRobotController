package org.firstinspires.ftc.teamcode.biobuzz.opmode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.HashMap;

public class BioBuzzDashboard {

    //alliances
    public static final int ALLIANCE_BLUE = 1;
    public static final int ALLIANCE_RED = 2;

    //the size of the field in inches
    public static final double FIELD_SIZE_INCHES = 141.5;


    //auto start poses
    public static final Pose2D RED_AUDIENCE_AUTO_START_POSE = new Pose2D(DistanceUnit.INCH, 40.25, 7, AngleUnit.DEGREES, 180); //54.25, 7.25, 90
    public static final Pose2D RED_GARDEN_POSE = new Pose2D(DistanceUnit.INCH, 11, 7.5, AngleUnit.DEGREES, 180);
    public static final Pose2D RED_SCORE_AUTO_START_POSE = new Pose2D(DistanceUnit.INCH, 57, FIELD_SIZE_INCHES - 7, AngleUnit.DEGREES, -90);

    public static final Pose2D BLUE_AUDIENCE_AUTO_START_POSE = new Pose2D(DistanceUnit.INCH, 57, FIELD_SIZE_INCHES - 7, AngleUnit.DEGREES, -90);
    public static final Pose2D BLUE_GARDEN_POSE = new Pose2D(DistanceUnit.INCH, 11, 7.5, AngleUnit.DEGREES, 180);
    public static final Pose2D BLUE_SCORE_AUTO_START_POSE = new Pose2D(DistanceUnit.INCH, 40.25, 7, AngleUnit.DEGREES, 180);

    //auto park poses
    public static final Pose2D RED_AUDIENCE_AUTO_PARK_POSE = new Pose2D(DistanceUnit.INCH, 13, 90, AngleUnit.DEGREES, 90);
    public static final Pose2D RED_SCORE_AUTO_PARK_POSE = new Pose2D(DistanceUnit.INCH, 15, FIELD_SIZE_INCHES - 22, AngleUnit.DEGREES, 0);

    public static final Pose2D BLUE_AUDIENCE_AUTO_PARK_POSE = new Pose2D(DistanceUnit.INCH, 13, FIELD_SIZE_INCHES - 22, AngleUnit.DEGREES, 0);
    public static final Pose2D BLUE_SCORE_AUTO_PARK_POSE = new Pose2D(DistanceUnit.INCH, 15, 90, AngleUnit.DEGREES, 90);

    //shooting target poses
    public static final Pose2D RED_AUDIENCE_TARGET_POSE = new Pose2D(DistanceUnit.INCH, 56, 60, AngleUnit.DEGREES, 0);
    public static final Pose2D RED_SCORE_TARGET_POSE = new Pose2D(DistanceUnit.INCH, 56, FIELD_SIZE_INCHES - 60, AngleUnit.DEGREES, 0);

    public static final Pose2D BLUE_AUDIENCE_TARGET_POSE = new Pose2D(DistanceUnit.INCH, 56, FIELD_SIZE_INCHES - 60, AngleUnit.DEGREES, 0);
    public static final Pose2D BLUE_SCORE_TARGET_POSE = new Pose2D(DistanceUnit.INCH, 56, 60, AngleUnit.DEGREES, 0);


    //The IMU reset pose for TeleOp
    public static final Pose2D RED_IMU_RESET_POSE = new Pose2D(DistanceUnit.INCH, 36, 63.67, AngleUnit.DEGREES, 0);
    public static final Pose2D BLUE_IMU_RESET_POSE = new Pose2D(DistanceUnit.INCH, 36, 63.67, AngleUnit.DEGREES, 0);


    //use the dashboard to save and read auto end
    public static final String AUTO_END_X_KEY = "X";
    public static final String AUTO_END_Y_KEY = "Y";
    public static final String AUTO_END_HEADING_KEY = "Heading";

    public static void saveAutoEndPose(HashMap<String, Object> blackboard, Pose2D pose)
    {
        //might be wrong pose
        if(pose.getX(DistanceUnit.INCH) < 1. && pose.getX(DistanceUnit.INCH) < 1)
            return;

        blackboard.put(AUTO_END_X_KEY, pose.getX(DistanceUnit.INCH));
        blackboard.put(AUTO_END_Y_KEY, pose.getY(DistanceUnit.INCH));
        blackboard.put(AUTO_END_HEADING_KEY, pose.getHeading(AngleUnit.DEGREES));
    }

    public static Pose2D robotAutoEndPose(HashMap<String, Object> blackboard)
    {
        double x = 0;
        double y = 0;
        double heading = 0;

        Object OX = blackboard.getOrDefault(AUTO_END_X_KEY, 0.0);
        Object OY = blackboard.getOrDefault(AUTO_END_Y_KEY, 0.0);
        Object OHeading = blackboard.getOrDefault(AUTO_END_HEADING_KEY, 0.0);

        try {
            if(OX != null)
                x = (double) OX;
        }
        catch(Exception ignored)
        {
        }

        try {
            if(OY != null)
                y = (double) OY;
        }
        catch(Exception ignored)
        {
        }

        try {
            if(OHeading != null)
                heading = (double) OHeading;
        }
        catch(Exception ignored)
        {
        }

        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
    }

}

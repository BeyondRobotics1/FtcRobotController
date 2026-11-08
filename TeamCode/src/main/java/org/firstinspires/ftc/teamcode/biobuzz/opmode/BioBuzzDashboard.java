package org.firstinspires.ftc.teamcode.biobuzz.opmode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.HashMap;

public class BioBuzzDashboard {

    //alliances
    public static final int kAllianceBlue = 1;
    public static final int kAllianceRed = 2;

    //the size of the field in inches
    public static final double kFieldSize = 141.5;


    //auto start poses
    public static final Pose2D kRedAutoStartPoseAudience = new Pose2D(DistanceUnit.INCH, 56, 60, AngleUnit.DEGREES, 0);
    public static final Pose2D kRedAutoStartPoseScore = new Pose2D(DistanceUnit.INCH, 56, kFieldSize - 60, AngleUnit.DEGREES, 0);

    public static final Pose2D kBlueAutoStartPoseAudience = new Pose2D(DistanceUnit.INCH, 56, kFieldSize - 60, AngleUnit.DEGREES, 0);
    public static final Pose2D kBlueAutoStartPoseScore = new Pose2D(DistanceUnit.INCH, 56, 60, AngleUnit.DEGREES, 0);

    //auto park poses
    public static final Pose2D kRedAutoParkPoseAudience = new Pose2D(DistanceUnit.INCH, 56, 60, AngleUnit.DEGREES, 0);
    public static final Pose2D kRedAutoParkPoseScore = new Pose2D(DistanceUnit.INCH, 56, kFieldSize - 60, AngleUnit.DEGREES, 0);

    public static final Pose2D kBlueAutoParkPoseAudience = new Pose2D(DistanceUnit.INCH, 56, kFieldSize - 60, AngleUnit.DEGREES, 0);
    public static final Pose2D kBlueAutoParkPoseScore = new Pose2D(DistanceUnit.INCH, 56, 60, AngleUnit.DEGREES, 0);

    //shooting target poses
    public static final Pose2D kRedTargetPoseAudience = new Pose2D(DistanceUnit.INCH, 56, 60, AngleUnit.DEGREES, 0);
    public static final Pose2D kRedTargetPoseScore = new Pose2D(DistanceUnit.INCH, 56, kFieldSize - 60, AngleUnit.DEGREES, 0);

    public static final Pose2D kBlueTargetPoseAudience = new Pose2D(DistanceUnit.INCH, 56, kFieldSize - 60, AngleUnit.DEGREES, 0);
    public static final Pose2D kBlueTargetPoseScore = new Pose2D(DistanceUnit.INCH, 56, 60, AngleUnit.DEGREES, 0);


    //The IMU reset pose for TeleOp
    public static final Pose2D kRedIMUResetPoseAudience = new Pose2D(DistanceUnit.INCH, 56, 60, AngleUnit.DEGREES, 0);
    public static final Pose2D kRedIMUResetPoseScore = new Pose2D(DistanceUnit.INCH, 56, kFieldSize - 60, AngleUnit.DEGREES, 0);

    public static final Pose2D kBlueIMUResetAudience = new Pose2D(DistanceUnit.INCH, 56, kFieldSize - 60, AngleUnit.DEGREES, 0);
    public static final Pose2D kBlueIMUResetPoseScore = new Pose2D(DistanceUnit.INCH, 56, 60, AngleUnit.DEGREES, 0);



    //use the dashboard to save and read auto end
    public static final String X = "X";
    public static final String Y = "Y";
    public static final String HEADING = "Heading";

    public static void saveAutoEndPose(HashMap<String, Object> blackboard, Pose2D pose)
    {
        //might be wrong pose
        if(pose.getX(DistanceUnit.INCH) < 1. && pose.getX(DistanceUnit.INCH) < 1)
            return;

        blackboard.put(X, pose.getX(DistanceUnit.INCH));
        blackboard.put(Y, pose.getY(DistanceUnit.INCH));
        blackboard.put(HEADING, pose.getHeading(AngleUnit.DEGREES));
    }

    public static Pose2D robotAutoEndPose(HashMap<String, Object> blackboard)
    {
        double x = 0;
        double y = 0;
        double heading = 0;

        Object OX = blackboard.getOrDefault(X, 0.0);
        Object OY = blackboard.getOrDefault(Y, 0.0);
        Object OHeading = blackboard.getOrDefault(HEADING, 0.0);

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

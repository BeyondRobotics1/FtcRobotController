package org.firstinspires.ftc.teamcode.decode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class DecodeBlackBoard {

    public static final Pose2D BLUE_TARGET_POSE = new Pose2D(DistanceUnit.INCH, 0, 144, AngleUnit.DEGREES, 0);
    public static final Pose2D RED_TARGET_POSE = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    public static final String DEFAULT_X = "DefaultX";
    public static final String DEFAULT_Y = "DefaultY";
    public static final String DEFAULT_HEADING = "DefaultHeading";

    public static final String X = "X";
    public static final String Y = "Y";
    public static final String HEADING = "Heading";

//    LinearOpMode mode;
//    public DecodeBlackBoard(LinearOpMode mode)
//    {
//        this.mode = mode;
//    }


    public static void saveDefaultAutoEndPose(Pose2D defaultPos)
    {
        LinearOpMode.blackboard.put(DEFAULT_X, defaultPos.getX(DistanceUnit.INCH));
        LinearOpMode.blackboard.put(DEFAULT_Y, defaultPos.getY(DistanceUnit.INCH));
        LinearOpMode.blackboard.put(DEFAULT_HEADING, defaultPos.getHeading(AngleUnit.DEGREES));
    }

    public static void saveAutoEndPose(Pose2D pose)
    {
        LinearOpMode.blackboard.put(X, pose.getX(DistanceUnit.INCH));
        LinearOpMode.blackboard.put(Y, pose.getY(DistanceUnit.INCH));
        LinearOpMode.blackboard.put(HEADING, pose.getHeading(AngleUnit.DEGREES));
    }


    public static Pose2D robotAutoEndPose()
    {
        double x = 0;
        double y = 0;
        double heading = 0;

        Object defaultX = LinearOpMode.blackboard.getOrDefault(DEFAULT_X, 0);
        Object defaultY = LinearOpMode.blackboard.getOrDefault(DEFAULT_Y, 0);
        Object defaultHeading = LinearOpMode.blackboard.getOrDefault(DEFAULT_HEADING, 0);

        double dX = 0;
        double dY = 0;
        double dHeading = 0;

        try {
            if(defaultX != null)
                dX = (double) defaultX;
        }
        catch(Exception ignored)
        {
        }

        try {
            if(defaultY != null)
                dX = (double) defaultY;
        }
        catch(Exception ignored)
        {
        }

        try {
            if(defaultHeading != null)
                dHeading = (double) defaultHeading;
        }
        catch(Exception ignored)
        {
        }

        Object OX = LinearOpMode.blackboard.getOrDefault(X, dX);
        Object OY = LinearOpMode.blackboard.getOrDefault(Y, dY);
        Object OHeading = LinearOpMode.blackboard.getOrDefault(HEADING, dHeading);

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

package org.firstinspires.ftc.teamcode.decode.OpMode;

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.HashMap;

public class DecodeBlackBoard {

    public static final int BLUE = 1;
    public static final int RED = 2;

    //the IMU reset pose for FAR TeleOp
    //Robot back to the wall and align with the loading zone white tape
    public static final Pose2D RED_FAR_RESET_POSE = new Pose2D(DistanceUnit.INCH, 133.75, 110.5, AngleUnit.DEGREES, 180); //133.5, 111
    public static final Pose2D BLUE_FAR_RESET_POSE = new Pose2D(DistanceUnit.INCH, 133.75, 30.875, AngleUnit.DEGREES, 180); //133.5, 30

    //The IMU reset pose for NEAR TeleOp
    //Robot front to ramp and align with gate handle
    public static final Pose2D RED_NEAR_RESET_POSE = new Pose2D(DistanceUnit.INCH, 18, 62.25, AngleUnit.DEGREES, -180); //
    public static final Pose2D BLUE_NEAR_RESET_POSE = new Pose2D(DistanceUnit.INCH, 18.115, 80.501, AngleUnit.DEGREES, 180); //18, 79.25

    //The start pose for NEAR Auto
    public static final Pose2D RED_NEAR_START_POSE = new Pose2D(DistanceUnit.INCH, 31.875, 11.5, AngleUnit.DEGREES, -90);//DecodeBlackBoard.RED_FAR_RESET_POSE;
    public static final Pose2D BLUE_NEAR_START_POSE = new Pose2D(DistanceUnit.INCH, 31.875, 130, AngleUnit.DEGREES, 90); //133.5, 30

    //the parking pose for NEAR Auto
    public static final Pose2D RED_NEAR_PARK_POSE = new Pose2D(DistanceUnit.INCH, 40, 61.5, AngleUnit.DEGREES, -180); //48, 69.5
    public static final Pose2D BLUE_NEAR_PARK_POSE = new Pose2D(DistanceUnit.INCH, 40, 80, AngleUnit.DEGREES, 180); //40, 83

    //The start pose for NEAR Auto
    public static final Pose2D RED_FAR_START_POSE = new Pose2D(DistanceUnit.INCH, 61.75, 134, AngleUnit.DEGREES, -180);//DecodeBlackBoard.RED_FAR_RESET_POSE;
    public static final Pose2D BLUE_FAR_START_POSE = new Pose2D(DistanceUnit.INCH, 61.75, 8.13, AngleUnit.DEGREES, 180); //55, 7.5, 180, 133.5, 30

    //the parking pose for FAR Auto
    public static final Pose2D RED_FAR_PARK_POSE = new Pose2D(DistanceUnit.INCH, 55, 110, AngleUnit.DEGREES, -90); //41, 56
    public static final Pose2D BLUE_FAR_PARK_POSE = new Pose2D(DistanceUnit.INCH, 60, 32, AngleUnit.DEGREES, 90); //40, 83


    //Open gate pose for NEAR TeleOp
    public static final Pose2D RED_OPEN_GATE_POSE = new Pose2D(DistanceUnit.INCH, 16, 84, AngleUnit.DEGREES, -155); //41, 56
    public static final Pose2D BLUE_OPEN_GATE_POSE = new Pose2D(DistanceUnit.INCH, 12, 59.5, AngleUnit.DEGREES, 150); //40, 83

    //Auto aiming target position (x, y coordinates for distance calculation)
    public static final Pose2D RED_TARGET_POSE = new Pose2D(DistanceUnit.INCH, 5, 5, AngleUnit.DEGREES, 0);
    public static final Pose2D BLUE_TARGET_POSE = new Pose2D(DistanceUnit.INCH, 5, 136.5, AngleUnit.DEGREES, 0); //5, 139

    public static final String DEFAULT_X = "DefaultX";
    public static final String DEFAULT_Y = "DefaultY";
    public static final String DEFAULT_HEADING = "DefaultHeading";

    public static final String X = "X";
    public static final String Y = "Y";
    public static final String HEADING = "Heading";

    public static final String Obelisk = "Obelisk";
    public static final int OBELISK_GPP = 21;
    public static final int OBELISK_PGP = 22;
    public static final int OBELISK_PPG = 23;

//    LinearOpMode mode;
//    public DecodeBlackBoard(LinearOpMode mode)
//    {
//        this.mode = mode;
//    }

    public static void saveObelisk(HashMap<String, Object> blackboard, int obelisk)
    {
        if(obelisk < OBELISK_GPP || obelisk > OBELISK_PPG)
            blackboard.put(Obelisk, OBELISK_GPP);
        else
            blackboard.put(Obelisk, obelisk);
    }

    public static int getObelisk(HashMap<String, Object> blackboard)
    {
        int result = OBELISK_GPP;

        Object ob = blackboard.getOrDefault(Obelisk, OBELISK_GPP);

        try {
            if(ob != null)
                result = (int) ob;
        }
        catch(Exception ignored)
        {
        }

        return result;
    }

    public static void saveDefaultAutoEndPose(HashMap<String, Object> blackboard, Pose2D defaultPos)
    {
        blackboard.put(DEFAULT_X, defaultPos.getX(DistanceUnit.INCH));
        blackboard.put(DEFAULT_Y, defaultPos.getY(DistanceUnit.INCH));
        blackboard.put(DEFAULT_HEADING, defaultPos.getHeading(AngleUnit.DEGREES));
    }

    public static void saveAutoEndPose(HashMap<String, Object> blackboard, Pose2D pose)
    {
        blackboard.put(X, pose.getX(DistanceUnit.INCH));
        blackboard.put(Y, pose.getY(DistanceUnit.INCH));
        blackboard.put(HEADING, pose.getHeading(AngleUnit.DEGREES));
    }


    public static Pose2D robotAutoEndPose(HashMap<String, Object> blackboard)
    {
        double x = 0;
        double y = 0;
        double heading = 0;

        Object defaultX = blackboard.getOrDefault(DEFAULT_X, 0);
        Object defaultY = blackboard.getOrDefault(DEFAULT_Y, 0);
        Object defaultHeading = blackboard.getOrDefault(DEFAULT_HEADING, 0);

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

        Object OX = blackboard.getOrDefault(X, dX);
        Object OY = blackboard.getOrDefault(Y, dY);
        Object OHeading = blackboard.getOrDefault(HEADING, dHeading);

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

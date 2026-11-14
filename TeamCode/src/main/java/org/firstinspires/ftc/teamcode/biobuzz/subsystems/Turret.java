package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.biobuzz.opmode.BioBuzzDashboard;

public class Turret {

    //PID controller for limelight 3A auto aiming
    PIDController controller;
    public static double kP = 0.0006;//0.001;// v1: 0.001;
    public static double kI = 0.003;//0.005;// v1: 0.25;
    public static double kD = 0;
    public static double kF = 0.000;//0.0005;//v1: 0.0095;
    public static double targetAngleDegree = 0;

    //hardware
    private LinearOpMode mode;
    private IMULocalizer localizer;
    private Servo turretPollen, turretNectar;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private int alliance = BioBuzzDashboard.ALLIANCE_RED;

    private boolean isAuto = true;

    // target x, y coordinates in INCH
    private double x0, y0;
    //robot current pos
    private double x, y, heading;

    private boolean isHeadingToGoal = false;


    //
    public static final double FULL_SERVO_RANGE_DEGREES = 367.34;
    public static final double HALF_SERVO_RANGE_DEGREES = FULL_SERVO_RANGE_DEGREES * 0.5;//138;//133.78;

    public static final double RIGHT_SERVO_POSITION = 0; //halfServoRangeDegrees, 0, 90 degree
    public static final double LEFT_SERVO_POSITION = 1;//-halfServoRangeDegrees; //0.380; //-90 degree
    public static final double CENTER_SERVO_POSITION = 0.5;//

    //servo positions for auto
    public static final double RED_AUDIENCE_AUTO_SERVO_POSITION = 0.25;//0.233
    public static final double RED_SCORE_AUTO_SERVO_POSITION = 0.5;
    public static final double BLUE_AUDIENCE_AUTO_SERVO_POSITION = 0.5;
    public static final double BLUE_SCORE_AUTO_SERVO_POSITION = 0.25;

    //
    public static final double TURRET_CENTER_OFFSET_DEGREES = 0;

    public static final double SERVO_DIRECTION = 1;

    /*
     * Turret pivot locations relative to the Pinpoint tracking center.
     * Positive forward is toward the robot's front. Positive left is toward
     * the robot's left. Use a negative forward value for a rear-mounted turret.
     */
    public static final double POLLEN_TURRET_FORWARD_OFFSET_INCHES = -4.5;
    public static final double POLLEN_TURRET_LEFT_OFFSET_INCHES = 0.0;
    public static final double NECTAR_TURRET_FORWARD_OFFSET_INCHES = 5.166;
    public static final double NECTAR_TURRET_LEFT_OFFSET_INCHES = 0.0;


    public Turret(HardwareMap hardwareMap, LinearOpMode mode,
                  Pose2D robotPose,
                  int alliance,
                  boolean useLimeLight,
                  boolean isAuto) {

        this.isAuto = isAuto;

        this.alliance = alliance;

        this.mode = mode;

        //this.mode.telemetry = new MultipleTelemetry(this.mode.telemetry, dashboard.getTelemetry());
        controller = new PIDController(kP, kI, kD);

        turretPollen = hardwareMap.get(Servo.class, "pollenTurret");
        turretNectar = hardwareMap.get(Servo.class, "nectarTurret");

        //turretPollen.setDirection(Servo.Direction.REVERSE);
        //turretNectar.setDirection(Servo.Direction.REVERSE);

        localizer = new IMULocalizer(hardwareMap, mode, robotPose, alliance);


        setServoPosition(CENTER_SERVO_POSITION, CENTER_SERVO_POSITION);
    }

    public void autoAim(boolean enabled)
    {
        if(localizer == null || !localizer.update())
            return;

        x0 = localizer.getX0();
        y0 = localizer.getY0();

        x = localizer.getX();
        y = localizer.getY();
        heading = localizer.getHeading();

//        mode.telemetry.addData("IMU Heading", "%.3f",  heading);
//        mode.telemetry.addData("IMU x", "%.3f", x);
//        mode.telemetry.addData("IMU y", "%.3f", y);
//        mode.telemetry.addData("Hive x", "%.3f", x0);
//        mode.telemetry.addData("Hive y", "%.3f", y0);

        //isHeadingToGoal = localizer.isHeadingToGoal();

        if(enabled) {

//            if(!isHeadingToGoal)
//            {
//                mode.telemetry.addData("Bot Heading:", heading);
//                mode.telemetry.addLine("Bot is not heading to goal, auto aiming skipped");
//
//                return;
//            }

            AimSolution pollenAim = calculateAimSolution(
                    POLLEN_TURRET_FORWARD_OFFSET_INCHES,
                    POLLEN_TURRET_LEFT_OFFSET_INCHES);
            AimSolution nectarAim = calculateAimSolution(
                    NECTAR_TURRET_FORWARD_OFFSET_INCHES,
                    NECTAR_TURRET_LEFT_OFFSET_INCHES);

            setServoPosition(pollenAim.servoPosition, nectarAim.servoPosition);

//            mode.telemetry.addLine(" ");
//
//            mode.telemetry.addData("Pollen turret x", "%.3f", pollenAim.turretX);
//            mode.telemetry.addData("Pollen turret y", "%.3f", pollenAim.turretY);
//            mode.telemetry.addData("Pollen turret angle", "%.3f", pollenAim.turretAngleDegrees);
//            mode.telemetry.addData("Pollen servo position", "%.5f", pollenAim.servoPosition);
//
//            mode.telemetry.addData("Nectar turret x", "%.3f", nectarAim.turretX);
//            mode.telemetry.addData("Nectar turret y", "%.3f", nectarAim.turretY);
//            mode.telemetry.addData("Nectar turret angle", "%.3f", nectarAim.turretAngleDegrees);
//            mode.telemetry.addData("Nectar servo position", "%.5f", nectarAim.servoPosition);

            //mode.telemetry.addData("Distance to goal", "%.3f", localizer.getRobotDistanceToGoal());
            //mode.telemetry.addData("Robot Zone", localizer.getRobotZone().ordinal());
        }
    }

    public double getServoPosition(boolean isPollen)
    {
        if(!isPollen && turretNectar != null)
            return turretNectar.getPosition();

        if(isPollen && turretPollen != null )
            return turretPollen.getPosition();

        return CENTER_SERVO_POSITION;
    }

    public double getBotHeadingDegrees()
    {
        if(localizer != null)
            return localizer.getHeading();
        else
            return 0;
    }

    public boolean isHeadingToGoal() {
        return isHeadingToGoal;
    }

    public IMULocalizer.RobotZone getRobotZone() {
        if(localizer != null)
            return localizer.getRobotZone();
        else
            return IMULocalizer.RobotZone.MEDIUM_SHOOTING_ZONE;
    }

    public void setIMUPoseToRobotStartPose()
    {
        if(localizer != null)
            localizer.setIMUPoseToRobotStartPose();
    }

    public void setIMUPose(Pose2D robotPose)
    {
        if(localizer != null)
            localizer.setIMUPose(robotPose);
    }

    //set the turret to center heading
    //could be useful when auto aiming is not
    //working as expected
    public void resetTurretHeading()
    {
        setServoPosition(CENTER_SERVO_POSITION, CENTER_SERVO_POSITION);
    }


    //return the distance from robot to goal
    //could use it to adjust shooter speed
    public double distanceToGoal()
    {
        return localizer.getRobotDistanceToGoal();
    }

    public void setServoPosition(double pollenServoPosition,
                                 double nectarServoPosition)
    {
        if(turretPollen != null)
            turretPollen.setPosition(pollenServoPosition);

        if(turretNectar != null)
            turretNectar.setPosition(nectarServoPosition);
    }

    /**
     * Converts a turret pivot offset from robot coordinates into field
     * coordinates, then calculates the shortest turret rotation to the target.
     * This assumes heading 0 degrees points along field +X and positive heading
     * rotates counter-clockwise.
     */
    private AimSolution calculateAimSolution(double forwardOffset, double leftOffset) {
        double headingRadians = Math.toRadians(heading);
        double cosHeading = Math.cos(headingRadians);
        double sinHeading = Math.sin(headingRadians);

        double turretX = x
                + forwardOffset * cosHeading
                - leftOffset * sinHeading;
        double turretY = y
                + forwardOffset * sinHeading
                + leftOffset * cosHeading;

        double targetBearingDegrees = Math.toDegrees(
                Math.atan2(y0 - turretY, x0 - turretX));
        double turretAngleDegrees = normalizeDegrees(
                targetBearingDegrees - heading - TURRET_CENTER_OFFSET_DEGREES);
        double servoPosition = Range.clip(
                CENTER_SERVO_POSITION
                        + SERVO_DIRECTION * turretAngleDegrees / FULL_SERVO_RANGE_DEGREES,
                RIGHT_SERVO_POSITION,
                LEFT_SERVO_POSITION);

        return new AimSolution(turretX, turretY, turretAngleDegrees, servoPosition);
    }

    private static final class AimSolution {
        final double turretX;
        final double turretY;
        final double turretAngleDegrees;
        final double servoPosition;

        AimSolution(
                double turretX,
                double turretY,
                double turretAngleDegrees,
                double servoPosition) {
            this.turretX = turretX;
            this.turretY = turretY;
            this.turretAngleDegrees = turretAngleDegrees;
            this.servoPosition = servoPosition;
        }
    }

    private static double normalizeDegrees(double angle) {
        angle = (angle + 180.0) % 360.0;
        if (angle < 0.0) {
            angle += 360.0;
        }
        return angle - 180.0;
    }
}

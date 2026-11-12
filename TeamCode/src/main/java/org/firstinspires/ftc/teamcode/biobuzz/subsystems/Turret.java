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

    private int alliance = BioBuzzDashboard.kAllianceRed;

    private boolean isAuto = true;

    // target x, y coordinates in INCH
    private double x0, y0;
    //robot current pos
    private double x, y, heading;

    private boolean isHeadingToGoal = false;


    //
    public static final double kFullServoRangeDegrees = 367.34;
    public static final double kHalfServoRangeDegrees = kFullServoRangeDegrees * 0.5;//138;//133.78;

    public static final double kServoPositionRight = 0; //halfServoRangeDegrees, 0, 90 degree
    public static final double kServoPositionLeft = 1;//-halfServoRangeDegrees; //0.380; //-90 degree
    public static final double kServoPositionCenter = 0.51;//

    //servo positions for auto
    public static final double kServoPositionAutoRedAudience = 0.25;//0.233
    public static final double kServoPositionAutoRedScore = 0.5;
    public static final double kServoPositionAutoBlueAudience = 0.5;
    public static final double kServoPositionAutoBlueScore = 0.25;

    //
    public static final double kTurretCenterOffsetDegrees = 0;

    public static final double kServoDirection = 1;
    //public static final double kServoDirectionRed = 1;

    //double servoDirection;

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


        setServoPosition(kServoPositionCenter);
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

        mode.telemetry.addData("IMU Heading", "%.3f",  heading);
        mode.telemetry.addData("IMU x", "%.3f", x);
        mode.telemetry.addData("IMU y", "%.3f", y);
        mode.telemetry.addData("Hive x", "%.3f", x0);
        mode.telemetry.addData("Hive y", "%.3f", y0);

        //isHeadingToGoal = localizer.isHeadingToGoal();

        if(enabled) {

//            if(!isHeadingToGoal)
//            {
//                mode.telemetry.addData("Bot Heading:", heading);
//                mode.telemetry.addLine("Bot is not heading to goal, auto aiming skipped");
//
//                return;
//            }

            double dx = x0 - x;
            double dy = y0 - y;

            // Assumes Pinpoint heading 0° points along field +X.
            double targetBearingDegrees = Math.toDegrees(Math.atan2(dy, dx));

            double turretAngleDegrees = normalizeDegrees(
                    targetBearingDegrees
                            - heading
                            - kTurretCenterOffsetDegrees
            );

            double servoPosition = Range.clip(
                    kServoPositionCenter
                            + kServoDirection * turretAngleDegrees / kFullServoRangeDegrees,
                    0.0,
                    1.0
            );

            setServoPosition(servoPosition);

            //setServoPosition(servoPosition);

            /*
            double servoPositionCalibration = calibrateTurret();
            servoPosition = getServoPosition() + servoPositionCalibration;

            mode.telemetry.addData("Limelight calibrated servo position:", "%.5f", servoPosition);

            if (servoPosition > kServoPositionLeft) {
                servoPosition = kServoPositionLeft;
            } else if (servoPosition < kServoPositionRight) {
                servoPosition = kServoPositionRight;
            }

            setServoPosition(servoPosition);
             */

            mode.telemetry.addLine(" ");

            mode.telemetry.addData("Servo position set", "%.5f", servoPosition);

            //mode.telemetry.addData("Distance to goal", "%.3f", localizer.getRobotDistanceToGoal());
            //mode.telemetry.addData("Robot Zone", localizer.getRobotZone().ordinal());

            mode.telemetry.addLine(" ");
            mode.telemetry.addData("Delta x:", Math.abs(x - x0));
            mode.telemetry.addData("Delta y:", Math.abs(y - y0));
            mode.telemetry.addData("targetBearingDegrees:", targetBearingDegrees);
            mode.telemetry.addData("turretAngleDegrees:", turretAngleDegrees);



        }
    }

    public double getServoPosition()
    {
        if(turretNectar != null)
            return turretNectar.getPosition();
        else
            return turretPollen.getPosition();
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
        setServoPosition(kServoPositionCenter);
    }


    //return the distance from robot to goal
    //could use it to adjust shooter speed
    public double distanceToGoal()
    {
        return localizer.getRobotDistanceToGoal();
    }

    public void setServoPosition(double position)
    {
        if(turretPollen != null)
            turretPollen.setPosition(position);

        if(turretNectar != null)
            turretNectar.setPosition(position);
    }

    private static double normalizeDegrees(double angle) {
        angle = (angle + 180.0) % 360.0;
        if (angle < 0.0) {
            angle += 360.0;
        }
        return angle - 180.0;
    }
}

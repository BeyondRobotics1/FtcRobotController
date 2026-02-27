package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.decode.OpMode.DecodeBlackBoard;

import java.util.List;

@Config

public class Turret {
    //AnalogInput analogInput; //for future use

    //hardware
    LinearOpMode mode;
    IMULocalizer localizer;
    Servo turretLeft, turretRight;
    Limelight3A limelight;

    boolean isAuto = true;

    // target x, y coordinates in INCH
    double x0, y0;

    public static double servoPositionRight = 0; //90 degree
    public static double servoPositionLeft = 0.380;//0.42; //-90 degree
    public static double servoPositionMiddle = 0.197;//0.195;//0 degree
    public static double servoPreviousPositionCalibration = 0.0;//0 degree

    public static double servoPositionObeliskDetectionRedAlliance = 0.357;
    public static double servoPositionObeliskDetectionBlueAlliance = 0.042;

    public static double servoPositionAutoShootingRedAlliance = 0.296;
    public static double servoPositionAutoShootingBlueAlliance = 0.096;

    //Limelight 3A auto aiming target degree of Tx
    public static double TARGET_ANGLE_DEGREE_RED = 0.0;
    public static double TARGET_ANGLE_DEGREE_BLUE = 6.0;

    double servoPositionRedFarAuto = 0.15;
    double servoPositionBlueFarAuto = 0.25;

    int alliance = DecodeBlackBoard.BLUE;
    int targetTagID; //derived from alliance

    boolean isHeadingToGoal = false;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    //PID controller for limelight 3A auto aiming
    PIDController controller;
    public static double kP = 0.001;//0.001; //0.8
    public static double kI = 0.005;//0.25; //0.01
    public static double kD = 0;
    public static double kF = 0.0095;
    public static double targetAngleDegree = 0;

    public Turret(HardwareMap hardwareMap, LinearOpMode mode,
                  Pose2D robotPose, Pose2D targetPose,
                  int alliance, //DecodeBlackBoard.BLUE or DecodeBlackBoard.RED
                  boolean usePinpoint,
                  boolean useLimeLight,
                  boolean isAuto) {

        this.isAuto = isAuto;

        this.alliance = alliance;

        if(alliance == DecodeBlackBoard.BLUE)
            targetTagID = 20;
        else
            targetTagID = 24;

        this.mode = mode;

        this.mode.telemetry = new MultipleTelemetry(this.mode.telemetry, dashboard.getTelemetry());
        controller = new PIDController(kP, kI, kD);


        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        turretRight = hardwareMap.get(Servo.class, "turretRight");

        if(usePinpoint) {
            localizer = new IMULocalizer(hardwareMap, mode, robotPose, targetPose, alliance);
        }

        // target coordinates
        x0 = targetPose.getX(DistanceUnit.INCH);
        y0 = targetPose.getY(DistanceUnit.INCH);

        //use limelight
        if(useLimeLight) {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");

            //auto use pipeline 1
            if(isAuto) {
                if(this.alliance == DecodeBlackBoard.RED)
                    limelight.pipelineSwitch(1);
                else
                    limelight.pipelineSwitch(2);
            }
            else //teleop use pipeline 0
                limelight.pipelineSwitch(0);
            limelight.start();
        }

        if(!isAuto)
        {
            if(this.alliance == DecodeBlackBoard.RED) {
                targetAngleDegree = TARGET_ANGLE_DEGREE_RED;
            }
            else {
                targetAngleDegree = TARGET_ANGLE_DEGREE_BLUE;
            }
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

    //for out zone auto only
    public void setOutZoneAutoServoPosition(int alliance)
    {
        if(alliance == DecodeBlackBoard.BLUE)
            setServoPosition(servoPositionBlueFarAuto);
        else
            setServoPosition(servoPositionRedFarAuto);
    }

    public void setServoPosition(double position)
    {
        turretLeft.setPosition(position);

        if(turretRight != null)
            turretRight.setPosition(position);
    }

    public double getServoPosition()
    {
        return turretLeft.getPosition();
    }

    public double getBotHeadingDegrees()
    {
        if(localizer != null)
            return localizer.getHeading();
        else
            return 0;
    }

    public IMULocalizer.RobotZone getRobotZone() {
        if(localizer != null)
            return localizer.getRobotZone();
        else
            return IMULocalizer.RobotZone.MEDIUM_SHOOTING_ZONE;
    }

    public boolean isHeadingToGoal() {
        return isHeadingToGoal;
    }

    public void setIMUPoseToRobotStartPose()
    {
        if(localizer != null)
            localizer.setIMUPoseToRobotStartPose();
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
        if(localizer != null)
            localizer.resetIMUPose();
    }


    //return the distance from robot to goal
    //could use it to adjust shooter speed
    public double distanceToGoal()
    {
        return localizer.getRobotDistanceToGoal();
    }

    //use limelight3A tx to calibrate turret heading with servo
    public double calibrateTurret() {

        boolean calibrationCalculated = false;
        double servoPositionCalibration = 0.0;

        controller.setPID(kP, kI, kD);

        if (limelight != null) {
            LLResult result = limelight.getLatestResult();

            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {

                int tagID = fr.getFiducialId();
                double currentAngleDegree = result.getTx();

                //mode.telemetry.addData("tagID:", tagID);
                //mode.telemetry.addData("targetTagID:", targetTagID);

                mode.telemetry.addData("Limelight3A Tx (Degree):", "%.3f", currentAngleDegree);
                mode.telemetry.addData("Target Angle  (Degree):", "%.3f", targetAngleDegree);

                if (targetTagID == tagID) {

                    double pid = controller.calculate(currentAngleDegree, targetAngleDegree);

                    servoPositionCalibration = pid + kF;

                    servoPreviousPositionCalibration = servoPositionCalibration;

                    calibrationCalculated = true;

                    break;
                }
            }
        }

        //if tag not found, return previous calibration value
        //to avoid oscillating between 0 and servoPositionCalibration
        if (calibrationCalculated)
            return servoPositionCalibration;
        else {
            ////reset controller
            controller.reset();
            controller.setPID(kP, kI, kD);

            return servoPreviousPositionCalibration;
        }
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


    public void autoAim(boolean enabled)
    {
        if(localizer == null || !localizer.update())
            return;

        double x = localizer.getX();
        double y = localizer.getY();
        double heading = localizer.getHeading();

        mode.telemetry.addData("IMU Heading", "%.3f",  heading);
        mode.telemetry.addData("IMU x", "%.3f", x);
        mode.telemetry.addData("IMU y", "%.3f", y);

        isHeadingToGoal = localizer.isHeadingToGoal();

        if(enabled) {

            if(!isHeadingToGoal)
            {
                mode.telemetry.addData("Bot Heading:", heading);
                mode.telemetry.addLine("Bot is not heading to goal, auto aiming skipped");

                return;
            }

            if (alliance == DecodeBlackBoard.RED)
                heading = -heading;

            //unwrap into [0 360]
            if (heading < 0)
                heading += 360;

            // calculate alpha
            double alpha = Math.toDegrees(Math.atan2(Math.abs(x - x0), Math.abs(y - y0)));
            double theta;

            if (alliance == DecodeBlackBoard.RED)
                theta = -90 - alpha + heading;
            else
                theta = 90 + alpha - heading;

            // calculate servo position
            double servoPosition;

            double servoPositionCalibration = calibrateTurret();

            if (alliance == DecodeBlackBoard.RED)
                servoPosition = servoPositionLeft * (theta + 90.0) / 180;
            else
                servoPosition = servoPositionLeft * (theta + 90.0) / 180;

            mode.telemetry.addData("IMU location based servo position:", "%.5f", servoPosition);

            servoPosition += servoPositionCalibration;

            mode.telemetry.addData("Limelight calibrated servo position:", "%.5f", servoPosition);


            if (servoPosition > servoPositionLeft) {
                servoPosition = servoPositionLeft;
            } else if (servoPosition < servoPositionRight) {
                servoPosition = servoPositionRight;
            }

            setServoPosition(servoPosition);

            mode.telemetry.addData("Servo position set", "%.5f", servoPosition);


            mode.telemetry.addData("Distance to goal", "%.3f", localizer.getRobotDistanceToGoal());
            mode.telemetry.addData("Robot Zone", localizer.getRobotZone().ordinal());

//            mode.telemetry.addData("Delta x:", Math.abs(x - x0));
//            mode.telemetry.addData("Delta y:", Math.abs(y - y0));
//            mode.telemetry.addData("alpha:", alpha);
//            mode.telemetry.addData("theta:", theta);
        }
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

}

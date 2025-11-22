package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class Turret {

//    private double redGoalAngleNearDegree = 45;
//    private double redGoalAngleFarDegree = -45;
//    private double redObliskAngleDegree = 120;
//
//    private double blueGoalAngleNearDegree = -45;
//    private double blueGoalAngleFarDegree = 45;
//    private double blueObliskAngleDegree = -120;

    private Limelight3A limelight;
    private Servo turretLeft;//, turretRight;

    private PIDController controller;
    public static double kP = 0.01; //0.8
    public static double kI = 0; //0.01
    public static double kD = 0;
    public double targetAngleDegree = 0;

    LinearOpMode mode;
    private int targetTagID;

    public Turret(HardwareMap hardwareMap, LinearOpMode mode, int targetTagID)
    {
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        //turretRight = hardwareMap.get(Servo.class, "turretRight");

        turretLeft.setDirection(Servo.Direction.REVERSE);

        this.mode = mode;
        this.targetTagID = targetTagID;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        mode.telemetry.addLine("Limelight camera started");

        controller = new PIDController(kP, kI, kD);
    }

    public boolean isLimeLigh3ARunning()
    {
        return limelight.isRunning();
    }

    public int getObliskID()
    {
        int id = 0; //invalid tag id

        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {

                id = fr.getFiducialId();

                if(id == 21 || id == 22 || id == 23)
                    return id;
            }
        }

        return id;
    }

    public void autoAim()
    {
        if(targetTagID == 24)
            mode.telemetry.addData("Auto Aiming on RED tag ID: ", targetTagID);
        else
            mode.telemetry.addData("Auto Aiming on BLUE tag ID: ", targetTagID);

        controller.setPID(kP, kI, kD);

        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {

                int tagID = fr.getFiducialId();
                double currentAngle = result.getTx();

                mode.telemetry.addData("Detected ID: ", tagID);

                if(tagID == targetTagID) {

                    double pid = controller.calculate(currentAngle, 0);
                    turretLeft.setPosition(pid + 0.5);

                    mode.telemetry.addData("Target tag found", "ID: %d, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }
                else
                {
                    mode.telemetry.addData("Tag mismatch", "ID: %d, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }
            }
        }
        else {
            mode.telemetry.addLine("Tag not detected");
            turretLeft.setPosition(0.5);
        }
    }

    public void setServoPosition(double position)
    {
        turretLeft.setPosition(position);
    }
}

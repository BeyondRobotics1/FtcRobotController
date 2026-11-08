package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class BioBuzzCamera {

    Limelight3A limelight;
    LinearOpMode mode;

    public BioBuzzCamera(HardwareMap hardwareMap, LinearOpMode mode) {

        this.mode = mode;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        //limelight.pipelineSwitch(0);

        limelight.start();
    }

    //use limelight to detect the April Tag of the Obelisk
    //return:
    //  0 - Obelisk April Tag is not detected
    //  21, 22, 23 - Obelisk April Tag is detected
    public int detectTagID()
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


    //get limelight heading in degree
    public double getTx(int targetTagID)
    {
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();

            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {

                int tagID = fr.getFiducialId();
                double currentAngleDegree = result.getTx();

                mode.telemetry.addData("tagID:", tagID);
                mode.telemetry.addData("targetTagID:", targetTagID);

                mode.telemetry.addData("Limelight3A Tx (Degree):", "%.3f", currentAngleDegree);
                //mode.telemetry.addData("Target Angle  (Degree):", "%.3f", targetAngleDegree);

                if (targetTagID == tagID) {
                    return currentAngleDegree;
                }
            }
        }

        return 0.0;
    }
}

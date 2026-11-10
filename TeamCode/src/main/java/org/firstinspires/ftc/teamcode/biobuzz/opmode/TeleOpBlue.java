package org.firstinspires.ftc.teamcode.biobuzz.opmode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Alliance-specific configuration shared by the two Blue TeleOps. */
public abstract class TeleOpBlue extends BioBuzzTeleOpBase {
    @Override
    protected int alliance() {
        return BioBuzzDashboard.kAllianceBlue;
    }

    @Override
    protected String allianceName() {
        return "BLUE";
    }

    @Override
    protected Pose2D audienceParkPose() {
        return BioBuzzDashboard.kBlueAutoParkPoseAudience;
    }

    @Override
    protected Pose2D scoreParkPose() {
        return BioBuzzDashboard.kBlueAutoParkPoseScore;
    }

    @Override
    protected Pose2D imuResetPose() {
        return BioBuzzDashboard.kBlueIMUResetPose;
    }
}

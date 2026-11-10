package org.firstinspires.ftc.teamcode.biobuzz.opmode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Alliance-specific configuration shared by the two Red TeleOps. */
public abstract class TeleOpRed extends BioBuzzTeleOpBase {
    @Override
    protected int alliance() {
        return BioBuzzDashboard.kAllianceRed;
    }

    @Override
    protected String allianceName() {
        return "RED";
    }

    @Override
    protected Pose2D audienceParkPose() {
        return BioBuzzDashboard.kRedAutoParkPoseAudience;
    }

    @Override
    protected Pose2D scoreParkPose() {
        return BioBuzzDashboard.kRedAutoParkPoseScore;
    }

    @Override
    protected Pose2D imuResetPose() {
        return BioBuzzDashboard.kRedIMUResetPose;
    }
}

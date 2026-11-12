package org.firstinspires.ftc.teamcode.biobuzz.opmode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Alliance-specific configuration shared by the two Red TeleOps. */
public abstract class TeleOpRed extends BioBuzzTeleOpBase {
    @Override
    protected int alliance() {
        return BioBuzzDashboard.ALLIANCE_RED;
    }

    @Override
    protected String allianceName() {
        return "RED";
    }

    @Override
    protected Pose2D audienceParkPose() {
        return BioBuzzDashboard.RED_AUDIENCE_AUTO_PARK_POSE;
    }

    @Override
    protected Pose2D scoreParkPose() {
        return BioBuzzDashboard.RED_SCORE_AUTO_PARK_POSE;
    }

    @Override
    protected Pose2D imuResetPose() {
        return BioBuzzDashboard.RED_IMU_RESET_POSE;
    }
}

package org.firstinspires.ftc.teamcode.biobuzz.opmode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Alliance-specific configuration shared by the two Blue TeleOps. */
public abstract class TeleOpBlue extends BioBuzzTeleOpBase {
    @Override
    protected int alliance() {
        return BioBuzzDashboard.ALLIANCE_BLUE;
    }

    @Override
    protected String allianceName() {
        return "BLUE";
    }

    @Override
    protected Pose2D audienceParkPose() {
        return BioBuzzDashboard.BLUE_AUDIENCE_AUTO_PARK_POSE;
    }

    @Override
    protected Pose2D scoreParkPose() {
        return BioBuzzDashboard.BLUE_SCORE_AUTO_PARK_POSE;
    }

    @Override
    protected Pose2D imuResetPose() {
        return BioBuzzDashboard.BLUE_IMU_RESET_POSE;
    }
}

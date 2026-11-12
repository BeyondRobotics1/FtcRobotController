package org.firstinspires.ftc.teamcode.biobuzz.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.biobuzz.opmode.BioBuzzDashboard;
import org.firstinspires.ftc.teamcode.biobuzz.opmode.BioBuzzTeleOpBase;

@TeleOp(name = "BioBuzz TeleOp RED Test", group = "BioBuzz Test")
public class TeleOpRedTest extends BioBuzzTeleOpBase {
    @Override
    protected StartingSide fixedStartingSide() {
        return StartingSide.AUDIENCE;
    }

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
        return BioBuzzDashboard.RED_IMU_RESET_POSE;
    }

    @Override
    protected Pose2D scoreParkPose() {
        return BioBuzzDashboard.RED_IMU_RESET_POSE;
    }

    @Override
    protected Pose2D imuResetPose() {
        return BioBuzzDashboard.RED_IMU_RESET_POSE;
    }
}

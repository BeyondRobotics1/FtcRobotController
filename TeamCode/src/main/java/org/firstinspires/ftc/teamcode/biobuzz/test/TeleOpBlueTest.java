package org.firstinspires.ftc.teamcode.biobuzz.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.biobuzz.opmode.BioBuzzDashboard;
import org.firstinspires.ftc.teamcode.biobuzz.opmode.BioBuzzTeleOpBase;

@TeleOp(name = "BioBuzz TeleOp BLUE Test", group = "BioBuzz Test")
public class TeleOpBlueTest extends BioBuzzTeleOpBase {

    @Override
    protected StartingSide fixedStartingSide() {
        return StartingSide.SCORE;
    }

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
        return BioBuzzDashboard.BLUE_IMU_RESET_POSE;
    }

    @Override
    protected Pose2D scoreParkPose() {
        return BioBuzzDashboard.BLUE_IMU_RESET_POSE;
    }

    @Override
    protected Pose2D imuResetPose() {
        return BioBuzzDashboard.BLUE_IMU_RESET_POSE;
    }
}

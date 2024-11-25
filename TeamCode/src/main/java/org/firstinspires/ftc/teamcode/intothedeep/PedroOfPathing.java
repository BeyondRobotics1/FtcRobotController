package org.firstinspires.ftc.teamcode.intothedeep;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "PedoPathing", group = "Linear Opmode")
//@Disabled
public class PedroOfPathing extends LinearOpMode {
    private Follower follower;
    private Pose startPose = new Pose(0,0,0);
    private Pose secondPose = new Pose(20,0,0);
    private Pose thirdPose = new Pose(40,20,90);
    private PathChain cycleStackTo;
    public void runOpMode() throws InterruptedException {
        cycleStackTo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(secondPose)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(secondPose),new Point(thirdPose)))
                .setLinearHeadingInterpolation(0,90)
                .setPathEndTimeoutConstraint(0)
                .build();
    }


}


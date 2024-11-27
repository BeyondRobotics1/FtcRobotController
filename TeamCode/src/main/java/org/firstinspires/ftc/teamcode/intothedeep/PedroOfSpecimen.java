package org.firstinspires.ftc.teamcode.intothedeep;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;


import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "PedroSpecimen", group = "Examples")
public class PedroOfSpecimen extends OpMode {
        private Timer pathTimer, actionTimer, opmodeTimer;
        private Follower follower;
        private int pathState = 1;
        private Pose startPose = new Pose(-64.5, -7.75, Math.toRadians(180));
        private Pose scorePose = new Pose(-36.5, -3, Math.toRadians(180));
        private Pose curvePoint = new Pose(-44, -3, Math.toRadians(180));
        private Pose sampleOne = new Pose(-40.5, -33, Math.toRadians(-55));
        private Pose sampleTwo = new Pose(-42.5, -43, Math.toRadians(-50));
        private Pose sampleThree = new Pose(-42.5, -53, Math.toRadians(-50));
        private Pose grab = new Pose(-60, -31, Math.toRadians(180));
        private PathChain scorePathOne, pickOne, giveOne, pickTwo, giveTwo, pickThree, giveThree, pickScore;
        public void buildPaths(){
            scorePathOne = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .setPathEndTimeoutConstraint(0)
                    .build();
            pickOne = follower.pathBuilder()
                    .addPath(new BezierCurve(new Point(scorePose), new Point(curvePoint), new Point(sampleOne)))
                    .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(-55))
                    .setPathEndTimeoutConstraint(0)
                    .build();
            giveOne = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(sampleOne), new Point(sampleOne)))
                    .setConstantHeadingInterpolation(Math.toRadians(-145))
                    .build();
            pickTwo = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(sampleOne), new Point(sampleTwo)))
                    .setConstantHeadingInterpolation(Math.toRadians(-50))
                    .setPathEndTimeoutConstraint(0)
                    .build();
            giveTwo = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(sampleTwo), new Point(sampleTwo)))
                    .setConstantHeadingInterpolation(Math.toRadians(-145))
                    .setPathEndTimeoutConstraint(0)
                    .build();
            pickThree = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(sampleTwo), new Point(sampleThree)))
                    .setConstantHeadingInterpolation(Math.toRadians(-50))
                    .setPathEndTimeoutConstraint(0)
                    .build();
            giveThree = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(sampleThree), new Point(sampleThree)))
                    .setConstantHeadingInterpolation(Math.toRadians(-155))
                    .setPathEndTimeoutConstraint(0)
                    .build();
            pickScore = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(sampleThree), new Point(grab)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .setPathEndTimeoutConstraint(0)
                    .build();


        }
    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1:
                follower.followPath(scorePathOne);
                setPathState(2);
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(pickOne);
                    setPathState(3);
                    break;
                }
            case 3:
                if(pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(giveOne);
                    setPathState(4);
                    break;
                }
            case 4:
                if(pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(pickTwo);
                    setPathState(5);
                    break;
                }
            case 5:
                if(pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(giveTwo);
                    setPathState(6);
                    break;
                }
            case 6:
                if(pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(pickThree);
                    setPathState(7);
                    break;
                }
            case 7:
                if(pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(giveThree);
                    setPathState(8);
                    break;
                }
            case 8:
                if(pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(pickScore);
                    setPathState(9);
                    break;
                }



        }
    }
    @Override
    public void loop() {

        // These loop the actions and movement of the robot
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        follower.setMaxPower(0.4);
    }
    @Override
    public void start() {

    }
    @Override
    public void stop() {
    }

}
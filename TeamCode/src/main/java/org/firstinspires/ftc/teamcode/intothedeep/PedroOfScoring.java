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

/**
 * This is an example auto that showcases movement and control of three servos autonomously.
 * It is able to detect the team element using a huskylens and then use that information to go to the correct spike mark and backdrop position.
 * There are examples of different ways to build paths.
 * A custom action system have been created that can be based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 9/8/2024
 */

@Autonomous(name = "PedroScoring", group = "Examples")
public class PedroOfScoring extends OpMode {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Follower follower;
    private int pathState = 1;
    private Pose startPose = new Pose(-62.5, 41.5, 0);
    private Pose scorePose = new Pose(-56, 55, Math.toRadians(-46));
    private Pose firstSample = new Pose(-50.5, 49, 0);
    private Pose secondSample = new Pose(-50.5, 58, 0);
    private Pose thirdSample = new Pose(-48.5, 57, Math.toRadians(27.5));
    private Pose subsystem = new Pose(-8, 24, Math.toRadians(-90));
    private Pose curve = new Pose(-20, 48, 0);
    private PathChain scorePathOne, scorePathTwo, scorePathThree, scorePathFour, first, second, third, sub;

    public void buildPaths() {
        scorePathOne = follower.pathBuilder()
            .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
            .setConstantHeadingInterpolation(Math.toRadians(-46))
            .setPathEndTimeoutConstraint(0)
            .build();
        first = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(firstSample)))
                .setConstantHeadingInterpolation(0)
                .setPathEndTimeoutConstraint(0)
                .build();
        scorePathTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstSample), new Point(scorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(-46))
                .setPathEndTimeoutConstraint(0)
                .build();
        second = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(secondSample)))
                .setConstantHeadingInterpolation(0)
                .build();
        scorePathThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSample), new Point(scorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(-46))
                .setPathEndTimeoutConstraint(0)
                .build();
        third = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(thirdSample)))
                .setConstantHeadingInterpolation( Math.toRadians(27.5))
                .setPathEndTimeoutConstraint(0)
                .build();
        scorePathFour = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdSample), new Point(scorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(-46))
                .setPathEndTimeoutConstraint(0)
                .build();
        sub = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(curve), new Point(subsystem)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .setPathEndTimeoutConstraint(0)
                .build();

    }
    public void setPathState(int state){
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
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(first);
                    setPathState(3);
                }
                break;
            case 3:
                if(pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(scorePathTwo);
                    setPathState(4);
                }
                break;
            case 4:
                if(pathTimer.getElapsedTimeSeconds()>2){
                    follower.followPath(second);
                    setPathState(5);
                }
                break;
            case 5:
                if(pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(scorePathThree);
                    setPathState(6);
                }
                break;
            case 6:
                if(pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(third);
                    setPathState(7);
                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds()>2){
                    follower.followPath(scorePathFour);
                    setPathState(8);
                }
                break;
            case 8:
                if(pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(sub);
                    setPathState(9);
                }
                break;

        }
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
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

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        follower.setMaxPower(0.3);
    }

    /** This method is called continuously after Init while waiting for "play". **/


    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

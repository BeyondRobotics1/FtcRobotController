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

@Autonomous(name = "Timothy Pedro", group = "A")
public class TimothyPedro extends OpMode {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Follower follower;
    private int pathState = 1;
    private Pose startPose = new Pose(0,5,0);
    private Pose controlPoint = new Pose(7.5,5,0);
    private Pose sampleOne = new Pose(19,-16, Math.toRadians(-45));
    private Pose hpOne = new Pose(19, -19, Math.toRadians(-135));
    private Pose sampleTwo = new Pose(0, 0, 0);
    private Pose sampleThree = new Pose(0,0,0);
    private Pose grabPose = new Pose(0,0,0);
    private PathChain getOne, giveOne, getTwo, giveTwo, getThree, giveThree, grabOne, scoreOne, grabTwo, scoreTwo, grabThree, scoreThree, grabFour, scoreFour, grabFive, scoreFive;
    public void buildPaths(){
        getOne = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(controlPoint),new Point(sampleOne)))
                .setLinearHeadingInterpolation(0, Math.toRadians(-45),1)
                .setPathEndTimeoutConstraint(0)
                .build();
        giveOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleOne), new Point(hpOne)))
                .setConstantHeadingInterpolation(Math.toRadians(-135))
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
                follower.followPath(getOne);
                setPathState(2);
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(giveOne);
                    setPathState(0);
                    break;
                }
//            case 3:
//                if(pathTimer.getElapsedTimeSeconds()>3){
//                    follower.followPath(getTwo);
//                    setPathState(4);
//                    break;
//                }
//            case 4:
//                if(pathTimer.getElapsedTimeSeconds()>3){
//                    follower.followPath(giveTwo);
//                    setPathState(5);
//                    break;
//                }
//            case 5:
//                if(pathTimer.getElapsedTimeSeconds()>3){
//                    follower.followPath(getThree);
//                    setPathState(6);
//                    break;
//                }
//            case 6:
//                if(pathTimer.getElapsedTimeSeconds()>3){
//                    follower.followPath(giveThree);
//                    setPathState(7);
//                    break;
//                }
//            case 7:
//                if(pathTimer.getElapsedTimeSeconds()>3){
//                    follower.followPath(grabOne);
//                    setPathState(8);
//                    break;
//                }
//            case 8:
//                if(pathTimer.getElapsedTimeSeconds()>3){
//                    follower.followPath(scoreOne);
//                    setPathState(9);
//                    break;
//                }
//            case 9:
//                if(pathTimer.getElapsedTimeSeconds()>3){
//                    follower.followPath(grabTwo);
//                    setPathState(10);
//                    break;
//                }
//            case 10:
//                if(pathTimer.getElapsedTimeSeconds()>3){
//                    follower.followPath(scoreTwo);
//                    setPathState(11);
//                    break;
//                }
        }
    }
    @Override
    public void loop(){

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
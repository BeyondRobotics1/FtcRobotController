package org.firstinspires.ftc.teamcode.intothedeep.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

/**
 * This is an example auto that showcases movement and control of three servos autonomously.
 * It is able to detect the team element using a huskylens and then use that information to go to the correct spike mark and backdrop position.
 * There are examples of different ways to build paths.
 * A custom action system have been created that can be based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 9/8/2024
 */

@Autonomous(name = "PedroCircle", group = "Pedro Examples")
@Disabled
public class PedroOfCircle extends OpMode {

    private Follower follower;
    private Pose startPose = new Pose(0,0,0);
    private Pose secondPose = new Pose(113,0,0);
    private Pose thirdPose = new Pose(113,-89,Math.toRadians(-90));
    private Pose fourthPose = new Pose(17, -89, Math.toRadians(180));
    private Pose fifthPose = new Pose(18,0,0);
    private PathChain cycleStackTo;
    public void buildPaths() {
        cycleStackTo = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(secondPose)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(new Point(secondPose), new Point(thirdPose)))
                .setLinearHeadingInterpolation(0, Math.toRadians(-90))
                .addPath(new BezierCurve(new Point(thirdPose), new Point(fourthPose)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                .addPath(new BezierCurve(new Point(fourthPose), new Point(fifthPose)))
                .setLinearHeadingInterpolation(Math.toRadians(180), 0)
                .setPathEndTimeoutConstraint(0)
                .build();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the actions and movement of the robot
        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.5);
    }

    /** This method is called continuously after Init while waiting for "play". **/


    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {

        buildPaths();

        follower.followPath(cycleStackTo);

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

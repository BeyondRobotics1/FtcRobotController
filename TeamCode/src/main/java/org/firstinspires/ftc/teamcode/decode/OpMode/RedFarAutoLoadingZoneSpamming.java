package org.firstinspires.ftc.teamcode.decode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Trigger;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
@Disabled
@Autonomous(name = "Red Far Loading Zone Auto", group = "Decode")
public class RedFarAutoLoadingZoneSpamming extends LinearOpMode {

    //Hardware
    private Shooter shooter;
    private Intake intake;
    private Trigger trigger;
    private Turret turret;
    private Indexer indexer;
    private int obelisk_id = DecodeBlackBoard.OBELISK_PGP;
    //status
    //Shooter.ShootingLocation shootingLocation = Shooter.ShootingLocation.MEDIUM;

    final private int openTriggerWaitTime = 70; //70, open trigger wait time in ms
    final private int shootBallWaitTime = 600;  //450, 550, 600 shooting three balls wait time in ms
    final private int turretStabilizationWaitTime = 1000; //time to wait for the turret to stabilize

    private Timer pathTimer;
    //private Timer opmodeTimer;
    private int pickupLimit = 2;
    private int pickupCounter = 0;

    private int pathState = 0;

    Follower follower;

    /**
     * Start Pose of our robot
     */
    private final Pose startPose = new Pose(DecodeBlackBoard.RED_FAR_START_POSE.getX(DistanceUnit.INCH),
            DecodeBlackBoard.RED_FAR_START_POSE.getY(DistanceUnit.INCH),
            Math.toRadians(DecodeBlackBoard.RED_FAR_START_POSE.getHeading(AngleUnit.DEGREES))); // Start Pose of our robot.

    // park pose
    private final Pose parkPose = new Pose(DecodeBlackBoard.RED_FAR_PARK_POSE.getX(DistanceUnit.INCH),
            DecodeBlackBoard.RED_FAR_PARK_POSE.getY(DistanceUnit.INCH),
            Math.toRadians(DecodeBlackBoard.RED_FAR_PARK_POSE.getHeading(AngleUnit.DEGREES))); //40, 80


    private final Pose scorePose = new Pose(56, 121.5, Math.toRadians(-180)); // 55, 20.5, 115 Scoring Pose of our robot.


    //corner
    private final Pose pickup1Pose = new Pose(40, 131.5, Math.toRadians(-180)); //40, 131.5, -180 S
    private final Pose grab1Pose = new Pose(10.5, 132.5, Math.toRadians(-180)); // 10.5, 134, -180


    //lower spike
    private final Pose pickup3Pose = new Pose(41.125, 106, Math.toRadians(-180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose grab3Pose = new Pose(10, 107, Math.toRadians(-180)); //10, 35

    //lower spike
    private final Pose pickup2Pose = new Pose(41.125, 123, Math.toRadians(-180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose grab2Pose = new Pose(10, 123, Math.toRadians(-180)); //10, 35


    private Path scorePreload;
    private PathChain scorePickup1Grab1, grab1Score;
    private PathChain scorePickup3, pickup3Grab3, grab3Score;
    private PathChain scorePickup2, pickup2Grab2, grab2Score;
    private PathChain scorePark;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Initializing indexer");
        telemetry.update();
        indexer = new Indexer(hardwareMap, this);

        telemetry.addLine("Initializing shooter");
        shooter = new Shooter(hardwareMap, this, DecodeBlackBoard.RED);

        telemetry.addLine("Initializing intake");
        intake = new Intake(hardwareMap, this);

        telemetry.addLine("Initializing trigger");
        trigger = new Trigger(hardwareMap);
        trigger.close();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        turret = new Turret(hardwareMap, this, new Pose2D(DistanceUnit.INCH,
                startPose.getX(), startPose.getY(), AngleUnit.DEGREES, startPose.getHeading()),
                DecodeBlackBoard.RED_TARGET_POSE,
                DecodeBlackBoard.RED,
                false,true, true);

        turret.setServoPosition(Turret.servoPositionObeliskDetectionRedAllianceFar);
        telemetry.addLine("hardware initialization completed");

        DecodeBlackBoard.saveAutoEndPose(blackboard, new Pose2D(DistanceUnit.INCH,
                parkPose.getX(), parkPose.getY(), AngleUnit.DEGREES, Math.toDegrees(parkPose.getHeading())));

        telemetry.addLine("initializing pedro pathing follower");
        pathTimer = new Timer();


        buildPaths();

        telemetry.addLine("initializing pedro pathing follower completed");


        telemetry.addLine("initializing LynxModule");
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry.addLine("LynxModule initialized");

        //waitForStart();
        while (!isStarted() && !isStopRequested()) {

            int tag_id = turret.detectObeliskTagID();

            telemetry.addLine("Red Far Auto");
            telemetry.addData("Obelisk ID:", tag_id);

            if (tag_id == DecodeBlackBoard.OBELISK_GPP) {
                obelisk_id = tag_id;
                telemetry.addLine("Obelisk: GPP");
            }
            else if(tag_id == DecodeBlackBoard.OBELISK_PGP) {
                obelisk_id = tag_id;
                telemetry.addLine("Obelisk: PGP");
            }
            else if(tag_id == DecodeBlackBoard.OBELISK_PPG) {
                obelisk_id = tag_id;
                telemetry.addLine("Obelisk: PPG");
            }
            else
                telemetry.addLine("Obelisk: Not Detected");

            telemetry.update();
            sleep(100);
        }


        turret.setServoPosition(Turret.servoShootingPositionRedFarAuto);

        shooter.setShootingLocation(Shooter.ShootingLocation.AUTO_FAR);
        shooter.setPower(0.9);
        sleep(150);//Flywheel need time to rotate up (0.4, 700)

        //let the PID work for a while
        for (int i = 0; i < 20; i++) {
            shooter.doFlyWheelVelocityPID();
            sleep(15);//100
        }

        setPathState(0);

        while (!isStopRequested() && opModeIsActive()) {
            hubs.forEach(LynxModule::clearBulkCache);

            follower.update();

            autonomousPathUpdate();

            //displayPose();

            shooter.doFlyWheelVelocityPID();

            saveAutoState();
        }

        //in the end save current robot pose into black board
        //saveAutoState();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                //wait for turret to stabilize
                if (pathTimer.getElapsedTime() > turretStabilizationWaitTime) {
                    pathTimer.resetTimer();

                    trigger.open();
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > openTriggerWaitTime) {//110
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.MEDIUM_FEED);

                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTime() > shootBallWaitTime)
                {
                    pathTimer.resetTimer();
                    setPathState(10);
                }
                break;


            //low spike
            case 10:

                trigger.close();
                intake.intake(0.95, 0.925);

                //move to the pickup 1 position
                follower.followPath(scorePickup3, true); //grabPickup1

                setPathState(11);

                break;
            case 11:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    //grab balls at position 2
                    follower.followPath(pickup3Grab3, true); //grabPickup1
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    //move grab1 position to open gate position
                    follower.followPath(grab3Score, true);
                    intake.intake(0.95, 0.0);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTime() > 500) { //for robot to stabilize
                    pathTimer.resetTimer();
                    trigger.open();
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTime() > openTriggerWaitTime) {//110, 300
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.MEDIUM_FEED);

                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTime() > shootBallWaitTime) {//110, 300
                    pathTimer.resetTimer();
                    setPathState(20);
                }
                break;


            //loading zone
            case 20:
                trigger.close();
                intake.intake(0.95, 0.925);

                //move to the pickup 1 position
                follower.followPath(scorePickup1Grab1, true); //grabPickup1

                setPathState(21);
                break;
            case 21:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(22);
                }
                break;
            case 22:
                //intake for 1.5 seconds to make sure all 3 balls are in
                if (pathTimer.getElapsedTime() > 1500) {
                    follower.followPath(grab1Score, true);
                    intake.intake(0.95, 0.0);
                    setPathState(23);
                }
                break;
            case 23:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(24);
                }
                break;
            case 24:
                if (pathTimer.getElapsedTime() > 500) { //wait for robot to stabilize
                    pathTimer.resetTimer();
                    trigger.open();
                    setPathState(25);
                }
                break;
            case 25:
                if (pathTimer.getElapsedTime() > openTriggerWaitTime) {
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.MEDIUM_FEED);
                    setPathState(26);
                }
                break;
            case 26:
                if (pathTimer.getElapsedTime() > shootBallWaitTime) { //shoot balls

                    pickupCounter++;

                    if(pickupCounter >= pickupLimit)
                        setPathState(900);
                    else if (pickupCounter == 1)
                        setPathState(30);
                    else
                        setPathState(20);
                }
                break;

            //low spike
            case 30:

                trigger.close();
                intake.intake(0.95, 0.925);

                //move to the pickup 1 position
                follower.followPath(scorePickup2, true); //grabPickup1

                setPathState(31);

                break;
            case 31:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    //grab balls at position 2
                    follower.followPath(pickup2Grab2, true); //grabPickup1
                    setPathState(32);
                }
                break;
            case 32:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    //move grab1 position to open gate position
                    follower.followPath(grab2Score, true);
                    intake.intake(0.95, 0.0);
                    setPathState(33);
                }
                break;
            case 33:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(34);
                }
                break;
            case 34:
                if (pathTimer.getElapsedTime() > 500) { //for robot to stabilize
                    pathTimer.resetTimer();
                    trigger.open();
                    setPathState(35);
                }
                break;
            case 35:
                if (pathTimer.getElapsedTime() > openTriggerWaitTime) {//110, 300
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.MEDIUM_FEED);

                    setPathState(36);
                }
                break;
            case 36:
                if (pathTimer.getElapsedTime() > shootBallWaitTime) {//110, 300
                    pathTimer.resetTimer();
                    setPathState(20);
                }
                break;

            //park
            case 900:
                follower.followPath(scorePark, true);
                setPathState(1000);
                break;
            case 1000: //end of auto
                if (!follower.isBusy()) {
                    saveAutoState();
                    setPathState(-1);
                }
                break;
        }

    }

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1Grab1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addPath(new BezierLine(pickup1Pose, grab1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), grab1Pose.getHeading())
                .build();

        grab1Score = follower.pathBuilder()
                .addPath(new BezierLine(grab1Pose, pickup1Pose))
                .setLinearHeadingInterpolation(grab1Pose.getHeading(), pickup1Pose.getHeading())
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();


        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();
        pickup3Grab3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, grab3Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), grab3Pose.getHeading())
                .build();
        grab3Score = follower.pathBuilder()
                .addPath(new BezierLine(grab3Pose, scorePose))
                .setLinearHeadingInterpolation(grab3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();
        pickup2Grab2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, grab2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), grab2Pose.getHeading())
                .build();
        grab2Score = follower.pathBuilder()
                .addPath(new BezierLine(grab2Pose, scorePose))
                .setLinearHeadingInterpolation(grab2Pose.getHeading(), scorePose.getHeading())
                .build();


        /* This is our scoreParkPathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    void setPathState(int newPathState) {
        this.pathState = newPathState;
    }

    void saveAutoState()
    {
        //pedro pos is in Radian
        Pose p = follower.getPose();
        DecodeBlackBoard.saveAutoEndPose(blackboard, new Pose2D(DistanceUnit.INCH,
                p.getX(), p.getY(), AngleUnit.DEGREES, Math.toDegrees(p.getHeading())));
    }

    void displayPose()
    {
        Pose p = follower.getPose();
        telemetry.addData("X", p.getX());
        telemetry.addData("Y", p.getY());
        telemetry.addData("Heading",  Math.toDegrees(p.getHeading()));

        DecodeBlackBoard.saveAutoEndPose(blackboard, new Pose2D(DistanceUnit.INCH,
                p.getX(), p.getY(), AngleUnit.DEGREES, Math.toDegrees(p.getHeading())));

        telemetry.update();
    }
}
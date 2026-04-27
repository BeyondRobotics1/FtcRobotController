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
@Autonomous(name = "Red Near Indexing Auto", group = "Decode")
public class RedNearIndexingAuto extends LinearOpMode {

    //wait time in ms
    private final int indexingWaitTime = 200;
    private final int shootingOneBallWaitTime = 400;
    private final int transferringOneBallWaitTime = 300;
    private final int openingTriggerWaitTime = 50;

    //hardware
    private Shooter shooter;
    private Intake intake;
    private Trigger trigger;
    private Turret turret;
    private Indexer indexer;

    private int obelisk_id = -1;

    //status
    private Timer pathTimer;
    private int pathState = 0;

    //5000 clean up just in case there's balls in the indexer
    //4000 for two-ball indexing 2, 0, 1
    //3000 for two-ball indexing 2, 1, 0
    //2000 for one-ball indexing
    //1000 for no-ball indexing
    private int indexingPathState = 0;

    Follower follower;

    /**
     * Start Pose of our robot
     */
    private final Pose startPose = new Pose(30.5, 11, Math.toRadians(-90)); //tart Pose of our robot.
    private final Pose scorePose = new Pose(43, 43, Math.toRadians(-133)); // 43, 43, -134 Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    //Highest (First Set)
    private final Pose pickup1Pose = new Pose(43, 58, Math.toRadians(180)); // 48.5, 58 Highest (First Set) picking up start
    private final Pose grab1Pose = new Pose(19, 58, Math.toRadians(180)); // 17.5, 58Highest (First Set)  picking up end.
    private final Pose backout1Pose = new Pose(23, 65.5, Math.toRadians(180)); //24, 65.5
    private final Pose openGatePose = new Pose(19.2, 65.5, Math.toRadians(180)); //18, 65.5 //gate position
    private final Pose openGate2Pose = new Pose(18, 76, Math.toRadians(180));
    //Middle (Second Set)
    private final Pose pickup2Pose = new Pose(42.5, 82, Math.toRadians(180)); // 46, 83 Middle (Second Set) picking up start.
    private final Pose grab2Pose = new Pose(12.25, 82, Math.toRadians(180)); // 12, 82.5 Middle (Second Set) picking up end.
    private final Pose backout2Pose = new Pose(20, 82, Math.toRadians(180)); // 20, 82.5 Middle (Second Set) backout.
    private final Pose backout22Pose = new Pose(45, 75.5, Math.toRadians(180)); //42, 75.5
    //Lowest (Third Set)
    private final Pose pickup3Pose = new Pose(42.5, 105, Math.toRadians(180)); //44, 105 Lowest (Third Set) picking up start.
    private final Pose grab3Pose = new Pose(12.25, 105, Math.toRadians(180)); // 12, 105 Highest (First Set) picking up end.

    //park
    private final Pose parkPose = new Pose(41, 56, Math.toRadians(180)); // 41, 60Park pose.

    private Path scorePreload;
    private PathChain scorePickup1, pickup1Grab1, grab1Score;// grab1OpenGate, openGateScore;
    private PathChain scorePickup2, pickup2Grab2, grab2OpenGate, openGate2Score;//grab2Score;
    private PathChain scorePickup3, pickup3Grab3, grab3Score;
    private PathChain scorePark;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Initializing indexer");
        telemetry.update();
        indexer = new Indexer(hardwareMap, this);

        telemetry.addLine("Initializing shooter");
        shooter = new Shooter(hardwareMap, this, DecodeBlackBoard.RED);
        shooter.setShootingLocation(Shooter.ShootingLocation.NEAR);

        telemetry.addLine("Initializing intake");
        intake = new Intake(hardwareMap, this);

        telemetry.addLine("Initializing trigger");
        trigger = new Trigger(hardwareMap);
        trigger.close();


        turret = new Turret(hardwareMap, this, new Pose2D(DistanceUnit.INCH,
                startPose.getX(), startPose.getY(), AngleUnit.DEGREES, startPose.getHeading()),
                DecodeBlackBoard.RED_TARGET_POSE,
                DecodeBlackBoard.RED,
                false,
                true, true);
        turret.setServoPosition(Turret.servoPositionObeliskDetectionRedAllianceNear);

        telemetry.addLine("hardware initialization completed");

        telemetry.addLine("initializing pedro pathing follower");
        pathTimer = new Timer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
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

            telemetry.addLine("Red Near Indexing Auto");
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


        //turret.setServoPosition(Turret.servoPositionAutoShootingRedAlliance);
        turret.setServoPosition(Turret.servoPositionMiddle);

        shooter.setPower(0.4);

        setPathState(0);

        sleep(100);//500

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

                    trigger.open();
                    setPathState(2); //2
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 80) {//100
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.SLOW_FEED);

                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > 950) { //1250

                    trigger.close();
                    intake.intake(0.95, 0.925);

                    //move to the pickup 1 position
                    follower.followPath(scorePickup2, true); //grabPickup1

                    setPathState(11);
                }
                break;
            //second set of balls
            case 11:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    ////grab balls at position 1
                    follower.followPath(pickup2Grab2, true); //grabPickup1
                    setPathState(111);
                }
                break;
            case 111:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    ////move grab1 position to open gate position
                    follower.followPath(grab2OpenGate, true);
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTime() > 4500) { //4000
                    //move from open gate position to score position
                    follower.followPath(openGate2Score, true);
                    setPathState(13);

                    if(obelisk_id == DecodeBlackBoard.OBELISK_GPP)
                        indexingPathState = 2000; //PGP -> GPP, 1 indexing
                    else if(obelisk_id == DecodeBlackBoard.OBELISK_PGP)
                        indexingPathState = 1000; //PGP -> PGP, 0 indexing
                    else if(obelisk_id == DecodeBlackBoard.OBELISK_PPG)
                        indexingPathState = 4000; //PGP -> PPG, 2 indexing - 2, 0, 1
                    else
                        indexingPathState = 1000; //no indexing if not valid tag
                }
                break;
            case 13:
                if(indexAndShoot())
                {
                    //move to the pickup 1 position
                    follower.followPath(scorePickup1, true); //grabPickup1

                    setPathState(21);
                }
                break;

            //first set of balls
            case 21:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    ////grab balls at set 2
                    follower.followPath(pickup1Grab1, true); //grabPickup1
                    setPathState(22);
                }
                break;
            case 22:
                if (!follower.isBusy()) {
                    //move to score position
                    follower.followPath(grab1Score, true);
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
                    setPathState(23);
                    if(obelisk_id == DecodeBlackBoard.OBELISK_GPP)
                        indexingPathState = 3000; //PPG -> GPP, 2 indexing - 2, 1, 0
                    else if(obelisk_id == DecodeBlackBoard.OBELISK_PGP)
                        indexingPathState = 2000; //PPG -> PGP, 1 indexing
                    else if(obelisk_id == DecodeBlackBoard.OBELISK_PPG)
                        indexingPathState = 1000; //PPG -> PPG, 0 indexing
                    else
                        indexingPathState = 1000; //no indexing if not valid tag


                }
                break;
            case 23:
                if(indexAndShoot())
                {
                    //move to the pickup 3 position
                    follower.followPath(scorePickup3, true); //grabPickup1

                    setPathState(31);
                }
                break;

            //3rd set of balls
            case 31:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    //grab balls at position 1
                    follower.followPath(pickup3Grab3, true); //grabPickup1
                    setPathState(32);
                }
                break;
            case 32:
                if (!follower.isBusy()) {

                    pathTimer.resetTimer();

                    follower.followPath(grab3Score, true);
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
                    setPathState(33);
                }
                break;
            case 33:
                if(pathTimer.getElapsedTime() > 400)
                {
                    if(obelisk_id == DecodeBlackBoard.OBELISK_GPP)
                        indexingPathState = 1000; //GPP -> GPP, no indexing
                    else if(obelisk_id == DecodeBlackBoard.OBELISK_PGP)
                        indexingPathState = 4000; //GPP -> PGP, 2 indexing - 2, 0, 1
                    else if(obelisk_id == DecodeBlackBoard.OBELISK_PPG)
                        indexingPathState = 2000; //GPP -> PPG, 1 indexing
                    else
                        indexingPathState = 1000; //no indexing if not valid tag


                    setPathState(34);
                }
                break;
            case 34:
                if(indexAndShoot())
                {
                    indexingPathState = 5000; //clean up
                    setPathState(50);
                }
                break;
            case 50: //clean up
                if(indexAndShoot())
                {
                    setPathState(51);
                }
                break;
            case 51:
                follower.followPath(scorePark, true); //grabPickup1
                intake.setIntakeMode(Intake.IntakeMode.IDLE);//stop the intake
                trigger.close();
                setPathState(100);
                break;
            case 100: //end of auto
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

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();
        pickup1Grab1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, grab1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), grab1Pose.getHeading())
                .build();

//        grab1OpenGate = follower.pathBuilder()
//                //.addPath(new BezierLine(grab1Pose, scorePose))
//                .addPath(new BezierLine(grab1Pose, backout1Pose))
//                .setLinearHeadingInterpolation(grab1Pose.getHeading(), backout1Pose.getHeading())
//                .addPath(new BezierLine(backout1Pose, openGatePose))
//                .setLinearHeadingInterpolation(backout1Pose.getHeading(), openGatePose.getHeading())
//                .build();
        grab2OpenGate = follower.pathBuilder()
                .addPath(new BezierLine(grab2Pose, backout2Pose))
                .setLinearHeadingInterpolation(grab2Pose.getHeading(), backout2Pose.getHeading())
                .addPath(new BezierLine(backout2Pose, openGate2Pose))
                .setLinearHeadingInterpolation(backout2Pose.getHeading(), openGate2Pose.getHeading())
                .build();

//        openGateScore = follower.pathBuilder()
//                .addPath(new BezierLine(openGatePose, scorePose))
//                .setLinearHeadingInterpolation(openGatePose.getHeading(), scorePose.getHeading())
//                .build();
        openGate2Score = follower.pathBuilder()
                .addPath(new BezierLine(openGate2Pose, backout22Pose))
                .setLinearHeadingInterpolation(openGate2Pose.getHeading(), backout22Pose.getHeading())
                .addPath(new BezierLine(backout22Pose, scorePose))
                .setLinearHeadingInterpolation(backout22Pose.getHeading(), scorePose.getHeading())
                .build();


        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();
        pickup2Grab2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, grab2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), grab2Pose.getHeading())
                .build();
        grab1Score = follower.pathBuilder()
                .addPath(new BezierLine(grab1Pose, scorePose))
                .setLinearHeadingInterpolation(grab1Pose.getHeading(), scorePose.getHeading())
                .build();
//        grab2Score = follower.pathBuilder()
//                .addPath(new BezierLine(grab2Pose, backout2Pose))
//                .setLinearHeadingInterpolation(grab2Pose.getHeading(), backout2Pose.getHeading())
//                .addPath(new BezierLine(backout2Pose, scorePose))
//                .setLinearHeadingInterpolation(backout2Pose.getHeading(), scorePose.getHeading())
//                .build();


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

        /* This is our scoreParkPathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }


    void setPathState(int newPathState) {
        this.pathState = newPathState;
    }

    boolean indexAndShoot()
    {
        if(indexingPathState >= 5000)
            return clearIndexer();
        else if(indexingPathState >= 4000)
            return doTwoIndexingShooting201();
        else if(indexingPathState >= 3000)
            return doTwoIndexingShooting210();
        else if (indexingPathState >= 2000)
            return doOneIndexingShooting();
        else if (indexingPathState >= 1000)
            return doNoneIndexingShooting();
        else
            return true;
    }

    /**
     * No indexing required
     */
    boolean doNoneIndexingShooting()
    {

        if(indexingPathState == 1000 ) {
            if (!follower.isBusy()) {

                pathTimer.resetTimer();
                trigger.open();
                indexingPathState = 1001;
            }
        }
        else if(indexingPathState == 1001 )
        {
            //open trigger takes 50 mm
            //shoot the third ball first
            //Feed the balls slower than reqular
            if (pathTimer.getElapsedTime() > openingTriggerWaitTime)
            {
                intake.setIntakeMode(Intake.IntakeMode.SLOW_FEED);
                pathTimer.resetTimer();

                indexingPathState = 1002;
            }
        }
        else if(indexingPathState == 1002 )
        {
            //shooting three takes 900 ms
            if (pathTimer.getElapsedTime() > 900)
            {
                trigger.close();
                intake.intake(0.95,0.925);

                pathTimer.resetTimer();

                indexingPathState = 1003;
                return true;
            }
        }

        return false;
    }

    /**
     * Index one artifact
     */
    boolean doOneIndexingShooting()
    {
        if(indexingPathState == 2000 ) {

            intake.setIntakeMode(Intake.IntakeMode.IDLE);
            trigger.close();

            //index the first ball
            pathTimer.resetTimer();
            indexer.index(1);

            indexingPathState = 2001;
        }
        else if(indexingPathState == 2001 )
        {
            //indexing takes 200 ms
            //then move the second ball up
            if (!follower.isBusy() &&
                    pathTimer.getElapsedTime() > indexingWaitTime)
            {
                intake.intake(0.5);
                trigger.open();

                pathTimer.resetTimer();
                indexingPathState = 2002;
            }
        }
        else if(indexingPathState == 2002 )
        {
            //open trigger takes 50 mm
            //shoot the second & third ball first
            if (pathTimer.getElapsedTime() > openingTriggerWaitTime)
            {
                intake.setIntakeMode(Intake.IntakeMode.SLOW_FEED);
                pathTimer.resetTimer();

                indexingPathState = 2003;
            }
        }
        else if(indexingPathState == 2003 )
        {
            //shooting two balls takes 600 ms
            //then un-index the first ball
            if (pathTimer.getElapsedTime() > 600)
            {
                intake.intake(0.5);
                indexer.index(0);

                pathTimer.resetTimer();
                indexingPathState = 2004;
            }
        }
        else if(indexingPathState == 2004 )
        {
            //un-indexing takes 200 ms, increased the waiting time
            //to allow other two balls to get out of the goal
            //then shoot the second ball
            if (pathTimer.getElapsedTime() > 2*indexingWaitTime)
            {
                intake.setIntakeMode(Intake.IntakeMode.SLOW_FEED);

                pathTimer.resetTimer();
                indexingPathState = 2005;
            }
        }
        else if(indexingPathState == 2005 )
        {
            //shooting takes 300 ms
            if (pathTimer.getElapsedTime() > shootingOneBallWaitTime)
            {
                indexer.index(0);
                trigger.close();

                indexingPathState = 2006;
                return true;
            }
        }

        return false;
    }

    /**
     * Index two artifact
     */
    boolean doTwoIndexingShooting210()
    {
        if(indexingPathState == 3000 ) {

            intake.setIntakeMode(Intake.IntakeMode.IDLE);
            trigger.close();

            //index the first ball
            pathTimer.resetTimer();
            indexer.index(1);

            indexingPathState = 3001;
        }
        else if(indexingPathState == 3001 )
        {
            //indexing takes 200 ms
            //then move the second ball up
            if (pathTimer.getElapsedTime() > indexingWaitTime)
            {
                intake.intake(0.9);
                pathTimer.resetTimer();
                indexingPathState = 3002;
            }
        }
        else if(indexingPathState == 3002 )
        {
            //moving up takes 300 ms
            //then index the second ball
            if (pathTimer.getElapsedTime() > transferringOneBallWaitTime)
            {
                indexer.index(2);
                pathTimer.resetTimer();

                indexingPathState = 3003;
            }
        }
        else if(indexingPathState == 3003 )
        {
            //indexing takes 200 ms
            //then move the third ball up
            if (!follower.isBusy() &&
                    pathTimer.getElapsedTime() > indexingWaitTime)
            {
                intake.intake(0.5);
                trigger.open();

                pathTimer.resetTimer();

                indexingPathState = 3004;
            }
        }
        else if(indexingPathState == 3004 )
        {
            //open trigger takes 50 mm
            //shoot the third ball first
            if (pathTimer.getElapsedTime() > openingTriggerWaitTime)
            {
                intake.setIntakeMode(Intake.IntakeMode.FEED);
                pathTimer.resetTimer();

                indexingPathState = 3005;
            }
        }
        else if(indexingPathState == 3005 )
        {
            //shooting takes 300 ms
            //then un-index the second ball
            if (pathTimer.getElapsedTime() > shootingOneBallWaitTime)
            {
                intake.intake(0.5);
                indexer.index(1);

                pathTimer.resetTimer();
                indexingPathState = 3006;
            }
        }
        else if(indexingPathState == 3006 )
        {
            //un-indexing takes 200 ms
            //then shoot the second ball
            if (pathTimer.getElapsedTime() > indexingWaitTime)
            {
                intake.setIntakeMode(Intake.IntakeMode.FEED);

                pathTimer.resetTimer();
                indexingPathState = 3007;
            }
        }
        else if(indexingPathState == 3007 )
        {
            //shooting takes 300 ms
            //then un-index the first ball
            if (pathTimer.getElapsedTime() > shootingOneBallWaitTime)
            {
                //deque the third ball
                intake.intake(0.5);
                indexer.index(0);

                pathTimer.resetTimer();
                indexingPathState = 3008;
            }
        }
        else if(indexingPathState == 3008 )
        {
            //indexing takes 200 ms
            //then shoot the first ball
            if (pathTimer.getElapsedTime() > indexingWaitTime)
            {
                intake.setIntakeMode(Intake.IntakeMode.FEED);

                pathTimer.resetTimer();
                indexingPathState = 3009;
            }
        }
        else if(indexingPathState == 3009 )
        {
            //shooting takes 300 ms
            if (pathTimer.getElapsedTime() > shootingOneBallWaitTime)
            {
                trigger.close();
                intake.intake(0.95,0.925);

                pathTimer.resetTimer();

                indexingPathState = 3100;
                return true;
            }
        }

        return false;
    }

    /**
     * Index two artifact
     */
    boolean doTwoIndexingShooting201()
    {
        if(indexingPathState == 4000 ) {

            intake.setIntakeMode(Intake.IntakeMode.IDLE);
            trigger.close();

            //index the first ball
            pathTimer.resetTimer();
            indexer.index(1);

            indexingPathState = 4001;
        }
        else if(indexingPathState == 4001 )
        {
            //indexing takes 200 ms
            //then move the second ball up
            if (pathTimer.getElapsedTime() > indexingWaitTime)
            {
                intake.intake(0.9);
                pathTimer.resetTimer();
                indexingPathState = 4002;
            }
        }
        else if(indexingPathState == 4002 )
        {
            //moving up takes 300 ms
            //then index the second ball
            if (pathTimer.getElapsedTime() > transferringOneBallWaitTime)
            {
                indexer.index(2);
                pathTimer.resetTimer();

                indexingPathState = 4003;
            }
        }
        else if(indexingPathState == 4003 )
        {
            //indexing takes 200 ms
            //then move the third ball up
            if (!follower.isBusy() &&
                    pathTimer.getElapsedTime() > indexingWaitTime)
            {
                intake.intake(0.5);
                trigger.open();

                pathTimer.resetTimer();

                indexingPathState = 4004;
            }
        }
        else if(indexingPathState == 4004 )
        {
            //open trigger takes 50 mm
            //shoot the third ball first
            if (pathTimer.getElapsedTime() > openingTriggerWaitTime)
            {
                intake.setIntakeMode(Intake.IntakeMode.FEED);
                pathTimer.resetTimer();

                indexingPathState = 4005;
            }
        }
        else if(indexingPathState == 4005 )
        {
            //shooting takes 300 ms
            //then un-index the second ball
            if (pathTimer.getElapsedTime() > shootingOneBallWaitTime)
            {
                trigger.close();
                intake.intake(0.5);
                indexer.index(0);

                pathTimer.resetTimer();
                indexingPathState = 4006;
            }
        }
        else if(indexingPathState == 4006 )
        {
            //un-indexing takes two spots take 400 ms
            //then shoot the second ball
            if (pathTimer.getElapsedTime() > 2*indexingWaitTime)
            {
                trigger.open();

                pathTimer.resetTimer();
                indexingPathState = 4007;
            }
        }
        else if(indexingPathState == 4007 )
        {
            //open trigger takes 50 mm
            //shoot the third ball first
            if (pathTimer.getElapsedTime() > openingTriggerWaitTime)
            {
                intake.setIntakeMode(Intake.IntakeMode.FEED);
                pathTimer.resetTimer();

                indexingPathState = 4008;
            }
        }
        else if(indexingPathState == 4008 )
        {
            //shooting takes 300 ms
            //then un-index the first ball
            if (pathTimer.getElapsedTime() > shootingOneBallWaitTime)
            {
                //deque the third ball
                intake.intake(0.5);
                indexer.index(1);

                pathTimer.resetTimer();
                indexingPathState = 4009;
            }
        }
        else if(indexingPathState == 4009 )
        {
            //indexing takes 200 ms
            //then shoot the first ball
            if (pathTimer.getElapsedTime() > indexingWaitTime)
            {
                intake.setIntakeMode(Intake.IntakeMode.FEED);

                pathTimer.resetTimer();
                indexingPathState = 4010;
            }
        }
        else if(indexingPathState == 4010 )
        {
            //shooting takes 300 ms
            if (pathTimer.getElapsedTime() > shootingOneBallWaitTime)
            {
                /////reset indexer IMPORTANT
                indexer.index(0);

                trigger.close();
                intake.intake(0.95,0.925);

                pathTimer.resetTimer();

                indexingPathState = 4100;
                return true;
            }
        }

        return false;
    }

    /**
     * Index two artifact
     */
    boolean clearIndexer()
    {
        if(indexingPathState == 5000 )
        {
            trigger.open();
            intake.intake(0.5);
            indexer.index(2);

            pathTimer.resetTimer();

            indexingPathState = 5001;
        }
        else if(indexingPathState == 5001 )
        {
            if (pathTimer.getElapsedTime() > 2*indexingWaitTime)
            {
                intake.setIntakeMode(Intake.IntakeMode.FEED);
                pathTimer.resetTimer();

                indexingPathState = 5003;
            }
        }
        else if(indexingPathState == 5003 )
        {
            //shooting takes 300 ms
            //then un-index the second ball
            if (pathTimer.getElapsedTime() > shootingOneBallWaitTime)
            {
                intake.intake(0.5);
                indexer.index(1);

                pathTimer.resetTimer();
                indexingPathState = 5004;
            }
        }
        else if(indexingPathState == 5004 )
        {
            //un-indexing takes 200 ms
            //then shoot the second ball
            if (pathTimer.getElapsedTime() > indexingWaitTime)
            {
                intake.setIntakeMode(Intake.IntakeMode.FEED);

                pathTimer.resetTimer();
                indexingPathState = 5005;
            }
        }
        else if(indexingPathState == 5005 )
        {
            //shooting takes 300 ms
            //then un-index the first ball
            if (pathTimer.getElapsedTime() > shootingOneBallWaitTime)
            {
                //deque the third ball
                intake.intake(0.5);
                indexer.index(0);

                pathTimer.resetTimer();
                indexingPathState = 5006;
            }
        }
        else if(indexingPathState == 5006 )
        {
            //indexing takes 200 ms
            //then shoot the first ball
            if (pathTimer.getElapsedTime() > indexingWaitTime)
            {
                intake.setIntakeMode(Intake.IntakeMode.FEED);

                pathTimer.resetTimer();
                indexingPathState = 5007;
            }
        }
        else if(indexingPathState == 5007 )
        {
            //shooting takes 300 ms
            if (pathTimer.getElapsedTime() > shootingOneBallWaitTime)
            {
                indexer.index(0);

                trigger.close();
                intake.setIntakeMode(Intake.IntakeMode.IDLE);//stop the intake

                indexingPathState = 5100;
                return true;
            }
        }

        return false;
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

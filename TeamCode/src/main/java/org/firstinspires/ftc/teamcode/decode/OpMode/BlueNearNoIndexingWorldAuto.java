package org.firstinspires.ftc.teamcode.decode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Trigger;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "Blue Near NO Indexing Auto", group = "Decode")
public class BlueNearNoIndexingWorldAuto extends LinearOpMode {

    //Hardware
    private Shooter shooter;
    private Intake intake;
    private Trigger trigger;
    private Turret turret;
    private Indexer indexer;
    private Lift lift;

    private int openTriggerWaitTime;// = 70; //70, open trigger wait time in ms
    private int shootBallWaitTime;// = 500;  //450, 550, 600 shooting three balls wait time in ms

    //status
    private int obelisk_id = DecodeBlackBoard.OBELISK_PGP;
    private Timer pathTimer;
    private int pathState = 0;       //state machine's state
    private int openGateCounter = 0; //crease 1 when gate is opened
    private int openGateWaitTimeSpike;// = 1800; //Open gate after taking the second spike ball
    private int openGateWaitTimeSpam;// = 1100;  //Open gate spam
    private int openGateLimit = 2;        //how many times the gate should be opened
    private Intake.IntakeMode intakeFeedMode = Intake.IntakeMode.FEED;

    Follower follower;

    //Start Pose of our robot
    private final Pose startPose = new Pose(DecodeBlackBoard.BLUE_NEAR_START_POSE.getX(DistanceUnit.INCH),
            DecodeBlackBoard.BLUE_NEAR_START_POSE.getY(DistanceUnit.INCH),
            Math.toRadians(DecodeBlackBoard.BLUE_NEAR_START_POSE.getHeading(AngleUnit.DEGREES))); //30.5, 131, 90, Start Pose of our robot.

    // park pose
    private final Pose parkPose = new Pose(DecodeBlackBoard.BLUE_NEAR_PARK_POSE.getX(DistanceUnit.INCH),
            DecodeBlackBoard.BLUE_NEAR_PARK_POSE.getY(DistanceUnit.INCH),
            Math.toRadians(DecodeBlackBoard.BLUE_NEAR_PARK_POSE.getHeading(AngleUnit.DEGREES))); //40, 80

    //score pose
    private final Pose scorePose = new Pose(55, 86.5, Math.toRadians(180)); // 53, 80, 45, 96 Pose of our robot.

    //Highest (First Set)
    private final Pose pickup1Pose = new Pose(41.5, 84, Math.toRadians(180)); //43, 84 Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose grab1Pose = new Pose(17.25, 84, Math.toRadians(180)); //17.75, 83
    //following two poses are not used
    private final Pose backout1Pose = new Pose(21, 78, Math.toRadians(180)); //20, 78
    private final Pose openGate1Pose = new Pose(16, 76, Math.toRadians(180)); //16.6, 76

    //Middle (Second Set)
    private final Pose pickup2Pose = new Pose(43, 61.5, Math.toRadians(180)); // 43, 60, Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose grab2Pose = new Pose(10, 61.5, Math.toRadians(180)); //9.5, 60
    private final Pose backout2Pose = new Pose(18, 61, Math.toRadians(180)); //18, 61
    private final Pose openGate2Pose = new Pose(14, 63, Math.toRadians(180)); //14, 63 //gate position
    private final Pose openGateBackout2Pose = new Pose(28, 64, Math.toRadians(180)); //28, 64

    //open gate cycling
    private final Pose openGateSetupPose = new Pose(32, 61.5, Math.toRadians(160)); //32, 61.5, 160 Middle (Second Set) backout
    private final Pose openGateStartPose = new Pose(20, 61.5, Math.toRadians(155)); //20, 61.5, 150 //gate position
    private final Pose openGatePose = new Pose(11.5, 61.5, Math.toRadians(153)); //11.5, 61.5, 150 //gate position
    private final Pose openGateBackoutPose = new Pose(28, 64, Math.toRadians(160)); //28, 64, 160

    //Lowest (Third Set)
    private final Pose pickup3Pose = new Pose(42.5, 105, Math.toRadians(180)); //44, 105 Lowest (Third Set) picking up start.
    private final Pose grab3Pose = new Pose(12.25, 105, Math.toRadians(180)); // 12, 105 Highest (First Set) picking up end.

//    //loading zone
//    private final Pose pickup4Pose = new Pose(24, 128, Math.toRadians(135)); //24, 128, 135
//    private final Pose grab4Pose = new Pose(12.25, 132.5, Math.toRadians(180)); //12, 132.5, 180



    private Path scorePreload;
    private PathChain scorePickup1Grab1, scorePickup1, pickup1Grab1, grab1Score;
    private PathChain scorePickup2Grab2, grab2OpenGate2, openGate2Score;
    private PathChain scoreOpenGate, openGateScore;
    //private PathChain scorePickup3, pickup3Grab3, grab3Score;
    private PathChain scorePark;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Initializing indexer");
        telemetry.update();
        indexer = new Indexer(hardwareMap, this);

        telemetry.addLine("Initializing shooter");
        shooter = new Shooter(hardwareMap, this, DecodeBlackBoard.BLUE);

        telemetry.addLine("Initializing intake");
        intake = new Intake(hardwareMap, this);

        telemetry.addLine("Initializing trigger");
        trigger = new Trigger(hardwareMap);
        trigger.close();

        telemetry.addLine("Initializing lift");
        lift = new Lift(hardwareMap);

        turret = new Turret(hardwareMap, this,new Pose2D(DistanceUnit.INCH,
                startPose.getX(), startPose.getY(), AngleUnit.DEGREES, startPose.getHeading()),
                DecodeBlackBoard.BLUE_TARGET_POSE,
                DecodeBlackBoard.BLUE,
                false,true, true);
        turret.setServoPosition(Turret.servoPositionObeliskDetectionBlueAllianceNear);
        telemetry.addLine("hardware initialization completed");

        DecodeBlackBoard.saveAutoEndPose(blackboard, new Pose2D(DistanceUnit.INCH,
                parkPose.getX(), parkPose.getY(), AngleUnit.DEGREES, Math.toDegrees(parkPose.getHeading())));

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

            if(gamepad1.a) {
                openGateLimit = 2;
            }
            else if (gamepad1.b) {
                openGateLimit = 3;
            }

            int tag_id = turret.detectObeliskTagID();

            if(openGateLimit == 2)
                telemetry.addLine("BLUE Near NO Indexing 15-Ball Auto");
            else
                telemetry.addLine("BLUE Near NO Indexing 18-Ball Auto");


            telemetry.addLine("");

            telemetry.addData("Is Limelight running:", turret.isLimeLight3ARunning());
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

            telemetry.addLine("");

            telemetry.addLine("Gamepad1.A: 15 Balls");
            telemetry.addLine("Gamepad1.B: 18 Balls");

            telemetry.update();
            sleep(100);
        }

        turret.setServoPosition(Turret.servoShootingPositionBlueNearAuto);

        shooter.setShootingLocation(Shooter.ShootingLocation.AUTO_NEAR);
        shooter.setPower(0.9);

        if(openGateLimit == 2) {
            sleep(150);//150Flywheel need time to rotate up (0.4, 700)

            //let the PID work for a while
            for (int i = 0; i < 40; i++) { //40, 80
                shooter.doFlyWheelVelocityPID();
                sleep(15);//100
            }

            openGateWaitTimeSpike = 2000; //2000 Open gate after taking the second spike ball
            openGateWaitTimeSpam = 1800;  //1800 Open gate spam

            openTriggerWaitTime = 70;
            shootBallWaitTime = 550;

            intakeFeedMode = Intake.IntakeMode.FEED;
        }
        else {

            sleep(100);//150Flywheel need time to rotate up (0.4, 700)

            //let the PID work for a while
            for (int i = 0; i < 10; i++) {//30, 65
                shooter.doFlyWheelVelocityPID();
                sleep(10);//100
            }

            openGateWaitTimeSpike = 1100; //1650 Open gate after taking the second spike ball
            openGateWaitTimeSpam = 820;  //1100 Open gate spam

            openTriggerWaitTime = 55;
            shootBallWaitTime = 410;

            intakeFeedMode = Intake.IntakeMode.FEED;
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
                    intake.intake(0.95,0);
                    pathTimer.resetTimer();

                    trigger.open();
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > openTriggerWaitTime) {//70
                    pathTimer.resetTimer();
                    intake.setIntakeMode(intakeFeedMode);

                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > shootBallWaitTime) { //800

                    //shooter.setShootingLocation(Shooter.ShootingLocation.MEDIUM);
                    setPathState(21);
                }
                break;

            //Middle set of balls & open gate
            case 21:
                trigger.close();

                //move to the pickup2/grab2 position
                follower.followPath(scorePickup2Grab2, true); //scorePickup2

                intake.intake(0.95,0.925);

                setPathState(22);//11
                break;
            case 22:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    //move from grab2 position to open gate position
                    follower.followPath(grab2OpenGate2, true);

                    //stop the transfer motor to save battery
                    intake.intake(0.95, 0.0);
                    setPathState(23);
                }
                break;
            case 23:
                if (pathTimer.getElapsedTime() > openGateWaitTimeSpike) { //
                    //move from open gate position to score position
                    follower.followPath(openGate2Score, true);

                    setPathState(24);
                }
                break;
            case 24:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    trigger.open();
                    setPathState(25);
                }
                break;
            case 25:
                if (pathTimer.getElapsedTime() > openTriggerWaitTime) {//80
                    pathTimer.resetTimer();
                    intake.setIntakeMode(intakeFeedMode);
                    setPathState(26);
                }
                break;
            case 26:
                if (pathTimer.getElapsedTime() > shootBallWaitTime) {
                    setPathState(31);
                }
                break;

            //open gates loops
            case 31:
                pathTimer.resetTimer();
                trigger.close();
                follower.followPath(scoreOpenGate, true);
                intake.intake(0.95,0.925);

                setPathState(32);
                break;
            case 32:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(33);
                }
                break;
            case 33:
                intake.detectArtifactColors();
                if(pathTimer.getElapsedTime() > openGateWaitTimeSpam) //1750, 1100 for 3 opens
                {
                    intake.intake(0.95,0.0);

                    follower.followPath(openGateScore, true);

                    setPathState(34);
                }
                break;
            case 34:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    trigger.open();
                    setPathState(35);
                }
                break;
            case 35:
                if (pathTimer.getElapsedTime() > openTriggerWaitTime) {//80
                    pathTimer.resetTimer();
                    intake.setIntakeMode(intakeFeedMode);

                    setPathState(36);
                }
                break;

            case 36:
                if (pathTimer.getElapsedTime() > shootBallWaitTime) { //shoot balls

                    openGateCounter++;

                    if(openGateCounter >= openGateLimit)
                        setPathState(71);
                    else
                        setPathState(31);
                }
                break;

            //Highest set of balls
            case 71:
                trigger.close();
                intake.intake(0.95, 0.925);

                pathTimer.resetTimer();

                //grab balls at position 1
                if(openGateLimit == 2) {

                    follower.followPath(scorePickup1, true); //grabPickup1
                    setPathState(72);
                }
                else
                {
                    follower.followPath(scorePickup1Grab1, true);
                    setPathState(73);//grabPickup1
                }

                break;
            case 72:
                if (!follower.isBusy()) {
                    follower.followPath(pickup1Grab1,  true);
                    setPathState(73);
                    pathTimer.resetTimer();
                }
                break;

            case 73:
                if (!follower.isBusy()) {
                    follower.followPath(grab1Score,  true);
                    setPathState(74);
                    pathTimer.resetTimer();
                }
                break;

            case 74:
                if (!follower.isBusy()) {//should be 110 {
                    pathTimer.resetTimer();
                    trigger.open();
                    setPathState(75);
                }
                break;
            case 75:
                if (pathTimer.getElapsedTime() > openTriggerWaitTime) {//should be 80
                    pathTimer.resetTimer();
                    intake.setIntakeMode(intakeFeedMode);

                    setPathState(76);
                }
                break;
            case 76:
                if (pathTimer.getElapsedTime() > shootBallWaitTime) { //shoot balls

                    setPathState(900);
                }
                break;

            case 900:
                follower.followPath(scorePark, true); //score to park
                intake.setIntakeMode(Intake.IntakeMode.IDLE);//stop the intake
                trigger.close();
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

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1Grab1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addPath(new BezierLine(pickup1Pose, grab1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), grab1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        pickup1Grab1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, grab1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), grab1Pose.getHeading())
                .build();

        grab1Score = follower.pathBuilder()
                .addPath(new BezierLine(grab1Pose, scorePose))
                .setLinearHeadingInterpolation(grab1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2Grab2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addPath(new BezierLine(pickup2Pose, grab2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), grab2Pose.getHeading())
                .build();

        grab2OpenGate2 = follower.pathBuilder()
                .addPath(new BezierLine(grab2Pose, backout2Pose))
                .setLinearHeadingInterpolation(grab2Pose.getHeading(), backout2Pose.getHeading())
                .addPath(new BezierLine(backout2Pose, openGate2Pose))
                .setLinearHeadingInterpolation(backout2Pose.getHeading(), openGate2Pose.getHeading())
                .build();

        openGate2Score = follower.pathBuilder()
                .addPath(new BezierLine(openGate2Pose, openGateBackout2Pose))
                .setLinearHeadingInterpolation(openGate2Pose.getHeading(), openGateBackout2Pose.getHeading())
                .addPath(new BezierLine(openGateBackout2Pose, scorePose))
                .setLinearHeadingInterpolation(openGateBackout2Pose.getHeading(), scorePose.getHeading())
                .build();


        /* This is our gate spamming PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreOpenGate = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, openGateSetupPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), openGateSetupPose.getHeading())
                .addPath(new BezierLine(openGateSetupPose, openGateStartPose))
                .setLinearHeadingInterpolation(openGateSetupPose.getHeading(), openGateStartPose.getHeading())
                .addPath(new BezierLine(openGateStartPose, openGatePose))
                .setLinearHeadingInterpolation(openGateStartPose.getHeading(), openGatePose.getHeading())
                .build();


        openGateScore = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose, openGateBackoutPose))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), openGateBackoutPose.getHeading())
                .addPath(new BezierLine(openGateBackoutPose, scorePose))
                .setLinearHeadingInterpolation(openGateBackoutPose.getHeading(), scorePose.getHeading())
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

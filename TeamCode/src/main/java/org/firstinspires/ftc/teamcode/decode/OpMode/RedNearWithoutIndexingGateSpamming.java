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


@Autonomous(name = "Red Near Gate Spamming", group = "Decode")
public class RedNearWithoutIndexingGateSpamming extends LinearOpMode{

    //Hardware
    private Shooter shooter;
    private Intake intake;
    private Trigger trigger;
    private Turret turret;
    private Indexer indexer;
    private Lift lift;

    private int obelisk_id = DecodeBlackBoard.OBELISK_PGP;

    //status
    private Timer pathTimer;
    private int pathState = 0;

    Follower follower;

    /**
     * Start Pose of our robot
     */
    private final Pose startPose = new Pose(30.5, 11, Math.toRadians(-90)); //tart Pose of our robot.
    private final Pose scorePose = new Pose(43, 43, Math.toRadians(-135)); // 43, 43, -134 Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    //Highest (First Set)
    private final Pose pickup1Pose = new Pose(43, 58, Math.toRadians(180)); // 48.5, 58 Highest (First Set) picking up start
    private final Pose grab1Pose = new Pose(19, 58, Math.toRadians(180)); // 17.5, 58Highest (First Set)  picking up end.
    private final Pose backout1Pose = new Pose(23, 65.5, Math.toRadians(180)); //24, 65.5

    private final Pose openGatePose = new Pose(17, 76, Math.toRadians(180)); //18, 65.5 //gate position
    private final Pose openGate2Pose = new Pose(17, 77, Math.toRadians(180)); //14, 77, --170 //gate position
    private final Pose openGatePickupPose = new Pose(13.5, 84, Math.toRadians(-170)); //
    //18, 65.5 //gate position

    //Middle (Second Set)
    private final Pose pickup2Pose = new Pose(42.5, 82, Math.toRadians(180)); // 46, 83 Middle (Second Set) picking up start.
    private final Pose grab2Pose = new Pose(12.25, 82, Math.toRadians(180)); // 12, 82.5 Middle (Second Set) picking up end.
    private final Pose backout2Pose = new Pose(20, 82, Math.toRadians(180)); // 20, 82.5 Middle (Second Set) backout.


    private final Pose backout22Pose = new Pose(45, 75.5, Math.toRadians(180)); //42, 75.5

    //Lowest (Third Set)
    private final Pose pickup3Pose = new Pose(42.5, 105, Math.toRadians(180)); //44, 105 Lowest (Third Set) picking up start.
    private final Pose grab3Pose = new Pose(12.25, 105, Math.toRadians(180)); // 12, 105 Highest (First Set) picking up end.

    //loading zone
    private final Pose pickup4Pose = new Pose(24, 128, Math.toRadians(135)); //24, 128, 135
    private final Pose grab4Pose = new Pose(12.25, 132.5, Math.toRadians(180)); //12, 132.5, 180

    //park
    private final Pose parkPose = new Pose(41, 60, Math.toRadians(180)); // Park pose.

    private Path scorePreload;
    private PathChain scorePickup1, pickup1Grab1, grab1Score;
    private PathChain scorePickup2, pickup2Grab2, grab2OpenGate, openGateScore;
    private PathChain scoreOpenGate,openGatePickup, gatePickupScore;
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

        telemetry.addLine("Initializing lift");
        lift = new Lift(hardwareMap);

        turret = new Turret(hardwareMap, this, new Pose2D(DistanceUnit.INCH,
                startPose.getX(), startPose.getY(), AngleUnit.DEGREES, startPose.getHeading()),
                DecodeBlackBoard.RED_TARGET_POSE,
                DecodeBlackBoard.RED,
                false,
                true, true);
        telemetry.addLine("hardware initialization completed");

        DecodeBlackBoard.saveDefaultAutoEndPose(new Pose2D(DistanceUnit.INCH,
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

            int tag_id = turret.detectObeliskTagID();

            telemetry.addLine("Red Near Auto");
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

            displayPose();

            shooter.shoot();
        }

        //in the end save current robot pose into black board
        saveAutoState();
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
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 80) {//100
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.FEED);

                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > 800) { //1250

                    trigger.close();
                    intake.intake(0.925);

                    //move to the pickup 2 position
                    follower.followPath(scorePickup2, true); //scorePickup2

                    setPathState(11);//11
                }
                break;

            //Middle set of balls
            case 11:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    //grab balls at position 1
                    follower.followPath(pickup2Grab2, true); //grabPickup1
                    setPathState(111);
                }
                break;
            case 111:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    //move grab1 position to open gate position
                    follower.followPath(grab2OpenGate, true);
                    setPathState(112);
                }
                break;
            case 112:
                if (!follower.isBusy()) {
                    //Keep the gate open for 1 second
                    pathTimer.resetTimer();
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTime() > 400) { //350
                    //move from open gate position to score position
                    follower.followPath(openGateScore, true);
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    trigger.open();
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTime() > 80) {//110, 300
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.FEED);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTime() > 800) { //shoot balls 900
             ;
                    trigger.close();
//                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
                    intake.intake(0.925);
                    //move to the open gate position
                    follower.followPath(scoreOpenGate, true); //grabPickup1

                    setPathState(210);
                }
                break;
            case 210:
                if (!follower.isBusy()){
                    pathTimer.resetTimer();
                    setPathState(21);
                }
                break;

            //Pickup balls from gate
            case 21:
                if (pathTimer.getElapsedTime() > 300) {//Keep gate open 700
                    pathTimer.resetTimer();

                    //grab balls at position 1
                    follower.followPath(openGatePickup, true); //
                    setPathState(22);
                }
                break;
            case 22:
                if (pathTimer.getElapsedTime() > 3000) { //in take

                    follower.followPath(gatePickupScore, true);
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
                    setPathState(23);
                }
                break;
            case 23:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    trigger.open();
                    setPathState(24);
                }
                break;
            case 24:
                if (pathTimer.getElapsedTime() > 80) {//100
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.FEED);

                    setPathState(25);
                }
                break;

            case 25:
                if (pathTimer.getElapsedTime() > 800) { //shoot balls
                    trigger.close();
                    intake.intake(0.925);

                    //move to the pickup 1 position
                    follower.followPath(scorePickup1, true); //scoreOpenGate

                    setPathState(41);
                }
                break;


            //Pickup balls from gate
            case 30:
                if (!follower.isBusy()){
                    pathTimer.resetTimer();
                    setPathState(31);
                }
                break;
            case 31:
                if (pathTimer.getElapsedTime() > 400) {//Keep gate open 700
                    pathTimer.resetTimer();

                    //grab balls at position 1
                    follower.followPath(openGatePickup, true); //
                    setPathState(32);
                }
                break;
            case 32:
                if (pathTimer.getElapsedTime() > 1000) { //in take

                    follower.followPath(gatePickupScore, true);
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
                    setPathState(33);
                }
                break;
            case 33:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    trigger.open();
                    setPathState(34);
                }
                break;
            case 34:
                if (pathTimer.getElapsedTime() > 80) {//100
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.FEED);

                    setPathState(35);
                }
                break;

            case 35:
                if (pathTimer.getElapsedTime() > 800) { //shoot balls
                    trigger.close();
                    intake.intake(0.925);

                    //move to the pickup 1 position
                    follower.followPath(scorePickup1, true); //grabPickup1

                    setPathState(41);
                }
                break;

            //1st set of balls
            case 41:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    //grab balls at position 1
                    follower.followPath(pickup1Grab1, true); //grabPickup1
                    setPathState(42);
                }
                break;
            case 42:
                if (!follower.isBusy()) {
                    follower.followPath(grab1Score, true);
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
                    setPathState(43);
                }

                break;
            case 43:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    trigger.open();
                    setPathState(44);
                }
                break;
            case 44:
                if (pathTimer.getElapsedTime() > 80) {//should be 110
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.FEED);

                    setPathState(45);
                }
                break;
            case 45:
                if (pathTimer.getElapsedTime() > 800) { //shoot balls 1000

                    trigger.close();
                    intake.intake(0.925);

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

        grab1Score = follower.pathBuilder()
                .addPath(new BezierLine(grab1Pose, scorePose))
                .setLinearHeadingInterpolation(grab1Pose.getHeading(), scorePose.getHeading())
                .build();


        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();
        pickup2Grab2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, grab2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), grab2Pose.getHeading())
                .build();
        grab2OpenGate = follower.pathBuilder()
                .addPath(new BezierLine(grab2Pose, backout2Pose))
                .setLinearHeadingInterpolation(grab2Pose.getHeading(), backout2Pose.getHeading())
                .addPath(new BezierLine(backout2Pose, openGatePose))
                .setLinearHeadingInterpolation(backout2Pose.getHeading(), openGatePose.getHeading())
                .build();


        /* This is our gate spamming PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreOpenGate = follower.pathBuilder()
                //.addPath(new BezierLine(grab1Pose, scorePose))
                .addPath(new BezierLine(scorePose, backout22Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), backout22Pose.getHeading())
                .addPath(new BezierLine(backout22Pose, openGate2Pose))
                .setLinearHeadingInterpolation(backout22Pose.getHeading(), openGate2Pose.getHeading())
                .build();
        openGatePickup = follower.pathBuilder()
                .addPath(new BezierLine(openGate2Pose, openGatePickupPose))
                .setLinearHeadingInterpolation(openGate2Pose.getHeading(), openGatePickupPose.getHeading())
                .build();
        gatePickupScore = follower.pathBuilder()
                .addPath(new BezierLine(openGatePickupPose, backout22Pose))
                .setLinearHeadingInterpolation(openGatePickupPose.getHeading(), backout22Pose.getHeading())
                .addPath(new BezierLine(backout22Pose, scorePose))
                .setLinearHeadingInterpolation(backout22Pose.getHeading(), scorePose.getHeading())
                .build();

        openGateScore = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose, backout22Pose))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), backout22Pose.getHeading())
                .addPath(new BezierLine(backout22Pose, scorePose))
                .setLinearHeadingInterpolation(backout22Pose.getHeading(), scorePose.getHeading())
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
        DecodeBlackBoard.saveAutoEndPose(new Pose2D(DistanceUnit.INCH,
                p.getX(), p.getY(), AngleUnit.DEGREES, Math.toDegrees(p.getHeading())));
    }

    void displayPose()
    {
        Pose p = follower.getPose();
        telemetry.addData("X", p.getX());
        telemetry.addData("Y", p.getY());
        telemetry.addData("Heading",  Math.toDegrees(p.getHeading()));

        DecodeBlackBoard.saveAutoEndPose(new Pose2D(DistanceUnit.INCH,
                p.getX(), p.getY(), AngleUnit.DEGREES, Math.toDegrees(p.getHeading())));

        telemetry.update();
    }
}

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
import org.firstinspires.ftc.teamcode.decode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Trigger;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "Auto Red Near", group = "Decode")
public class RedNearAuto extends LinearOpMode {

   //Hardware
    private Shooter shooter;
    private Intake intake;
    private Trigger trigger;
    private Turret turret;
    private Indexer indexer;
    private Lift lift;

    //status
    private Timer pathTimer;
    private int pathState = 0;

    Follower follower;

    /**
     * Start Pose of our robot
     */
    private final Pose startPose = new Pose(17.25, 28.75, Math.toRadians(180)); //tart Pose of our robot.
    private final Pose scorePose = new Pose(52, 52, Math.toRadians(-135)); // 49, 47 Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    //x 44
    private final Pose pickup1Pose = new Pose(48.5, 58, Math.toRadians(180)); // 47.5, 58Highest (First Set) picking up start
    private final Pose grab1Pose = new Pose(18, 58, Math.toRadians(180)); // 17.5, 58Highest (First Set)  picking up end.
    private final Pose backout1Pose = new Pose(23, 65.5, Math.toRadians(180)); //24, 65.5
    private final Pose openGatePose = new Pose(19.35, 65.5, Math.toRadians(180)); //18, 65.5

    private final Pose pickup2Pose = new Pose(44, 82.5, Math.toRadians(180)); // 46, 83 Middle (Second Set) picking up start.
    private final Pose grab2Pose = new Pose(12, 82.5, Math.toRadians(180)); // 12, 83 Middle (Second Set) picking up end.
    private final Pose backout2Pose = new Pose(20, 82.5, Math.toRadians(180)); // 20, 83Middle (Second Set) backout.

    private final Pose pickup3Pose = new Pose(44, 105, Math.toRadians(180)); //46, 106 Lowest (Third Set) picking up start.
    private final Pose grab3Pose = new Pose(12, 105, Math.toRadians(180)); // Highest (First Set) picking up end.

    //negative? not possible, something wrong, but
    private final Pose pickup4Pose = new Pose(24, 128, Math.toRadians(135)); //-2, 120, 90, 24, 132.5, 180 loading zone
    private final Pose grab4Pose = new Pose(12, 132.5, Math.toRadians(180)); //-2, 136, 90 12, 132.5, 180

    private final Pose parkPose = new Pose(41, 60, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Path scorePreload;
    private PathChain scorePickup1, pickup1Grab1, grab1OpenGate, openGateScore;
    private PathChain scorePickup2, pickup2Grab2, grab2Score;
    private PathChain scorePickup3, pickup3Grab3, grab3Score;
    private PathChain scorePickup4, scoreGrab4, grab4Score;
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

        waitForStart();

        shooter.setPower(0.4);

        setPathState(0);

        sleep(100);//500

        while (!isStopRequested() && opModeIsActive()) {
            hubs.forEach(LynxModule::clearBulkCache);

            follower.update();
            autonomousPathUpdate();

            //turret.autoAim();
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
                if (pathTimer.getElapsedTime() > 100) {//110
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.FEED);

                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > 950) { //1250

                    trigger.close();
                    intake.intake(0.925);

                    //move to the pickup 1 position
                    follower.followPath(scorePickup1, true); //grabPickup1

                    setPathState(11);
                }
                break;
            //first set of balls
            case 11:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    //grab balls at position 1
                    follower.followPath(pickup1Grab1, true); //grabPickup1
                    setPathState(111);
                }
                break;
            case 111:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    //move grab1 position to open gate position
                    follower.followPath(grab1OpenGate, true);
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
                if (pathTimer.getElapsedTime() > 350) { //500, 650
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
                if (pathTimer.getElapsedTime() > 100) {//110, 300
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.FEED);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTime() > 950) { //shoot balls 1500

                    trigger.close();
                    intake.intake(0.925);

                    //move to the pickup 1 position
                    follower.followPath(scorePickup2, true); //grabPickup1

                    setPathState(21);
                }
                break;
            //second set of balls
            case 21:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    //grab balls at position 1
                    follower.followPath(pickup2Grab2, true); //grabPickup1
                    setPathState(22);
                }
                break;
            case 22:
                if (!follower.isBusy()) {
                    //move to backout position
                    //follower.followPath(grab2Backout2, true);
                    follower.followPath(grab2Score, true);
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
                if (pathTimer.getElapsedTime() > 100) {//110
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.FEED);

                    setPathState(25);
                }
                break;
            case 25:
                if (pathTimer.getElapsedTime() > 950) { //shoot balls 1500
                    trigger.close();
                    intake.intake(0.925);

                    //move to the pickup 1 position
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
                    follower.followPath(grab3Score, true);
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
                    setPathState(33);
                }

//                if (pathTimer.getElapsedTime() > 1300) { //grab 3 balls 2000
//                    //move to score position
//                    follower.followPath(grab3Score, true);
//                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
//                    setPathState(33);
//                }
                break;
            case 33:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    trigger.open();
                    setPathState(34);
                }
                break;
            case 34:
                if (pathTimer.getElapsedTime() > 100) {//should be 110
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.FEED);

                    setPathState(35);
                }
                break;
            case 35:
                if (pathTimer.getElapsedTime() > 950) { //shoot balls 1000

                    trigger.close();
                    intake.intake(0.925);

                    //move to the pickup 1 position
                    follower.followPath(scorePickup4, true); //grabPickup1

                    setPathState(41);
                }
                break;

            //4th set of balls
            case 41:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    follower.followPath(scoreGrab4, true); //grabPickup4

                    setPathState(411);
                }
                break;
            case 411:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    setPathState(42);
                }
                break;
            case 42:
                if (pathTimer.getElapsedTime() > 50) {

                    //grab balls at position 4 & back to score position
                    follower.followPath(grab4Score, true); //grab4Score, pickup4Score
                    setPathState(43);
                }
                break;
            case 43:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.FEED);
                    trigger.open();
                    setPathState(44);
                }
                break;
            case 44:
                if (pathTimer.getElapsedTime() > 50) {//should be 100
                    pathTimer.resetTimer();
                    //intake.setIntakeMode(Intake.IntakeMode.FEED);

                    setPathState(45);
                }
                break;
            case 45:
                if (pathTimer.getElapsedTime() > 900) { //shoot balls 1500
                    setPathState(50);
                }
                break;

            case 50:
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

        grab1OpenGate = follower.pathBuilder()
                //.addPath(new BezierLine(grab1Pose, scorePose))
                .addPath(new BezierLine(grab1Pose, backout1Pose))
                .setLinearHeadingInterpolation(grab1Pose.getHeading(), backout1Pose.getHeading())
                .addPath(new BezierLine(backout1Pose, openGatePose))
                .setLinearHeadingInterpolation(backout1Pose.getHeading(), openGatePose.getHeading())
                .build();
        openGateScore = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose, scorePose))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), scorePose.getHeading())
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

        grab2Score = follower.pathBuilder()
                .addPath(new BezierLine(grab2Pose, backout2Pose))
                .setLinearHeadingInterpolation(grab2Pose.getHeading(), backout2Pose.getHeading())
                .addPath(new BezierLine(backout2Pose, scorePose))
                .setLinearHeadingInterpolation(backout2Pose.getHeading(), scorePose.getHeading())
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

        /* This is our scorePickup4 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup4Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup4Pose.getHeading())
                .build();
        scoreGrab4 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup4Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup4Pose.getHeading())
                .addPath(new BezierLine(pickup4Pose, grab4Pose))
                .setLinearHeadingInterpolation(pickup4Pose.getHeading(), grab4Pose.getHeading())
                .build();

        grab4Score = follower.pathBuilder()
                .addPath(new BezierLine(grab4Pose, scorePose))
                .setLinearHeadingInterpolation(grab4Pose.getHeading(), scorePose.getHeading())
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
}

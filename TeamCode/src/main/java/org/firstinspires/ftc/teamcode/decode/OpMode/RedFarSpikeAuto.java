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
import org.firstinspires.ftc.teamcode.decode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Trigger;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "Red Far Spike", group = "Decode")
public class RedFarSpikeAuto extends LinearOpMode {
    //Hardware
    private Shooter shooter;
    private Intake intake;
    //private DriveTrain driveTrain;
    private Trigger trigger;
    private Turret turret;
    private Indexer indexer;
    private int obelisk_id = DecodeBlackBoard.OBELISK_PGP;
    //status
    //Shooter.ShootingLocation shootingLocation = Shooter.ShootingLocation.MEDIUM;

    private Timer pathTimer;
    //private Timer opmodeTimer;

    private int pathState = 0;

    Follower follower;

    /**
     * Start Pose of our robot
     */
    /**
     * Start Pose of our robot
     */
    private final Pose startPose = new Pose(55, 134, Math.toRadians(-90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(58, 120, Math.toRadians(-112)); // 55, 20.5, 90 Scoring Pose of our robot.

    private final Pose pickup1Pose = new Pose(40, 132, Math.toRadians(180)); // 40, 130 Second pickup spot
    private final Pose grab1Pose = new Pose(12.5, 134, Math.toRadians(180)); // 12.25, 134 Second pickup spot

    private final Pose pickup2Pose = new Pose(40, 126, Math.toRadians(180)); //Third pickup spot
    private final Pose grab2Pose = new Pose(12.5, 122, Math.toRadians(180)); // Second pickup spot

    //Lowest (Third Set)
    private final Pose pickup3Pose = new Pose(42.5, 105, Math.toRadians(180)); //44, 105 Lowest (Third Set) picking up start.
    private final Pose grab3Pose = new Pose(12.5, 105, Math.toRadians(180)); // 12, 105 Highest (First Set) picking up end.


    private final Pose parkPose = new Pose(38, 129 , Math.toRadians(-180)); // 55, 31.5, 90 Where we park

    private Path scorePreload;
    private PathChain scorePickup1Grab1, grab1Score;
    private PathChain scorePickup2Grab2, grab2Score;
    private PathChain scorePickup3, pickup3Grab3, grab3Score;
    private PathChain scorePark;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Initializing indexer");
        telemetry.update();
        indexer = new Indexer(hardwareMap, this);

        telemetry.addLine("Initializing shooter");
        shooter = new Shooter(hardwareMap, this, DecodeBlackBoard.BLUE);
        shooter.setShootingLocation(Shooter.ShootingLocation.OUT_ZONE);

        telemetry.addLine("Initializing intake");
        intake = new Intake(hardwareMap, this);

        telemetry.addLine("Initializing trigger");
        trigger = new Trigger(hardwareMap);
        trigger.close();

        turret = new Turret(hardwareMap, this, new Pose2D(DistanceUnit.INCH,
                startPose.getX(), startPose.getY(), AngleUnit.DEGREES, startPose.getHeading()),
                DecodeBlackBoard.BLUE_TARGET_POSE,
                DecodeBlackBoard.BLUE,
                false,
                true, true);
        turret.resetTurretHeading();
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

            telemetry.addLine("Blue Far Auto");
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
        //turret.setServoPosition(0.244);

        shooter.setPower(0.60);

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
                if (pathTimer.getElapsedTime() > 80) {//110
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.MEDIUM_FEED);

                    setPathState(10);
                }
                break;



            //first loop, loading zone
            case 10:
                if (pathTimer.getElapsedTime() > 1000) { //shoot spike 3

                    trigger.close();
                    intake.intake(0.925);

                    //move to the pickup 1 position
                    follower.followPath(scorePickup1Grab1, true); //grabPickup1

                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    //Keep intake on for 1 second
                    pathTimer.resetTimer();
                    setPathState(12);
                }
                break;
            case 12:
                //intake for 1.5 seconds to make sure all 3 balls are in
                if (pathTimer.getElapsedTime() > 1300) {
                    follower.followPath(grab1Score, true);
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
                if (pathTimer.getElapsedTime() > 500) {//wait for robot to stabilize
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.MEDIUM_FEED);
                    setPathState(60);
                }
                break;

            //low spike
            case 60:
                if (pathTimer.getElapsedTime() > 1000) { //shoot preload

                    trigger.close();
                    intake.intake(0.925);

                    //move to the pickup 1 position
                    follower.followPath(scorePickup3, true); //grabPickup1

                    setPathState(61);
                }
                break;
            case 61:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    //grab balls at position 2
                    follower.followPath(pickup3Grab3, true); //grabPickup1
                    setPathState(62);
                }
                break;
            case 62:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    //move grab1 position to open gate position
                    follower.followPath(grab3Score, true);
                    setPathState(63);
                }
                break;
            case 63:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    trigger.open();
                    setPathState(64);
                }
                break;
            case 64:
                if (pathTimer.getElapsedTime() > 80) {//110, 300
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.MEDIUM_FEED);
                    setPathState(20);
                }
                break;


            //loading zone again
            case 20:
                if (pathTimer.getElapsedTime() > 1000) { //1000

                    trigger.close();
                    intake.intake(0.925);

                    //move to the pickup 1 position
                    follower.followPath(scorePickup1Grab1, true); //grabPickup1

                    setPathState(21);
                }
                break;
            case 21:
                if (!follower.isBusy()) {
                    //Keep inke on for 1 second
                    pathTimer.resetTimer();
                    setPathState(22);
                }
                break;
            case 22:
                if (pathTimer.getElapsedTime() > 1300) { //intake balls
                    follower.followPath(grab1Score, true);
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
                if (pathTimer.getElapsedTime() > 500) {//wait for robot to stabilize
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.MEDIUM_FEED);
                    setPathState(30);
                }
                break;


            //third loop
            case 30:
                if (pathTimer.getElapsedTime() > 1000) { //1250

                    trigger.close();
                    intake.intake(0.925);

                    //move to the pickup 1 position
                    follower.followPath(scorePickup2Grab2, true); //grabPickup1

                    setPathState(31);
                }
                break;
            case 31:
                if (!follower.isBusy()) {
                    //Keep inke on for 1 second
                    pathTimer.resetTimer();
                    setPathState(32);
                }
                break;
            case 32:
                if (pathTimer.getElapsedTime() > 1300) { //intake balls
                    follower.followPath(grab2Score, true);
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
                if (pathTimer.getElapsedTime() > 500) {//wait for robot to stabilize
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.MEDIUM_FEED);
                    setPathState(40);
                }
                break;


            //forth loop by passed, no enough time
            case 40:
                if (pathTimer.getElapsedTime() > 1000) {

                    trigger.close();
                    intake.intake(0.925);

                    setPathState(900);

                    ////move to the pickup 1 position
                    //follower.followPath(scorePickup1Grab1, true); //grabPickup1

                    //setPathState(41);
                }
                break;
            case 41:
                if (!follower.isBusy()) {
                    //Keep intake on for 1 second
                    pathTimer.resetTimer();
                    setPathState(42);
                }
                break;
            case 42:
                //intake for 2 seconds to make sure all 3 balls are in
                if (pathTimer.getElapsedTime() > 1300) {
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
                if (pathTimer.getElapsedTime() > 500) {//wait for robot to stabilize
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.MEDIUM_FEED);
                    setPathState(100);
                }
                break;


            //park
            case 900:
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(scorePark, true);
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
                    trigger.close();
                    setPathState(1000);
                }
                break;
            case 1000: //end of auto
                if (!follower.isBusy()) {
                    saveAutoState();
                    setPathState(-1);
                }
                break;
            //first set of balls


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



        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2Grab2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addPath(new BezierLine(pickup2Pose, grab2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), grab2Pose.getHeading())
                .build();


        grab2Score = follower.pathBuilder()
                .addPath(new BezierLine(grab2Pose, pickup2Pose))
                .setLinearHeadingInterpolation(grab2Pose.getHeading(), pickup2Pose.getHeading())
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
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

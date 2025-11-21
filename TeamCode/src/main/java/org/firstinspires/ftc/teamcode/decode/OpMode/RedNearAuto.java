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

import org.firstinspires.ftc.teamcode.DriveTrain;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Trigger;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "Auto Red Near", group = "Decode")
public class RedNearAuto extends LinearOpMode {

    private Timer pathTimer;
    private Timer opmodeTimer;

    private Shooter shooter;
    private Intake intake;
    private DriveTrain driveTrain;
    private Trigger trigger;
    private Turret turret;

    Shooter.ShootingLocation shootingLocation = Shooter.ShootingLocation.Medium;


    private int pathState = 0;

    Follower follower;

    /**
     * Start Pose of our robot
     */
    private final Pose startPose = new Pose(16.5, 30, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(42, 42, Math.toRadians(-135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose pickup1Pose = new Pose(44, 59, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose grab1Pose = new Pose(19, 59, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup2Pose = new Pose(44, 83, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose grab2Pose = new Pose(19, 83, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup3Pose = new Pose(44, 106, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose grab3Pose = new Pose(19, 106, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose parkPose = new Pose(41, 58, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Path scorePreload;
    private PathChain scorePickup1, pickup1Grab1, grab1Score;
    private PathChain scorePickup2, pickup2Grab2, grab2Score;
    private PathChain scorePickup3, pickup3Grab3, grab3Score;
    private PathChain scorePark;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Initializing drive train");
        telemetry.update();
        driveTrain = new DriveTrain(hardwareMap, this, false);

        telemetry.addLine("Initializing shooter");
        shooter = new Shooter(hardwareMap, this);
        shooter.setShootingLocation(Shooter.ShootingLocation.Near);

        telemetry.addLine("Initializing intake");
        intake = new Intake(hardwareMap, this);

        telemetry.addLine("Initializing trigger");
        trigger = new Trigger(hardwareMap, this);
        trigger.close();

        turret = new Turret(hardwareMap, this, 24);
        telemetry.addLine("hardware initialization completed");

        telemetry.addLine("initializing pedro pathing follower");
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

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

        opmodeTimer.resetTimer();
        setPathState(0);

        sleep(500);

        while (!isStopRequested() && opModeIsActive()) {
            hubs.forEach(LynxModule::clearBulkCache);

            follower.update();
            autonomousPathUpdate();

            turret.autoAim();
            shooter.shoot();

        }
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
                if (pathTimer.getElapsedTime() > 300) {//should be 250, 1000 is for camera
                    pathTimer.resetTimer();
                    intake.SetIntakeMode(Intake.IntakeMode.FEED);

                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > 2000) { //make sure wait for all balls are shoot

                    trigger.close();
                    intake.intake(0.9);

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
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTime() > 1500) {
                    //move to score position
                    follower.followPath(grab1Score, true);
                    intake.intake(0);
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
                if (pathTimer.getElapsedTime() > 300) {//should be 250, 1000 is for camera
                    pathTimer.resetTimer();
                    intake.SetIntakeMode(Intake.IntakeMode.FEED);

                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTime() > 2000) {

                    trigger.close();
                    intake.intake(0.9);

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
                if (pathTimer.getElapsedTime() > 1500) {
                    //move to score position
                    follower.followPath(grab2Score, true);
                    intake.intake(0);
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
                if (pathTimer.getElapsedTime() > 300) {//should be 250, 1000 is for camera
                    pathTimer.resetTimer();
                    intake.SetIntakeMode(Intake.IntakeMode.FEED);

                    setPathState(25);
                }
                break;
            case 25:
                if (pathTimer.getElapsedTime() > 2000) {

                    trigger.close();
                    intake.intake(0.9);

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
                if (pathTimer.getElapsedTime() > 1500) {
                    //move to score position
                    follower.followPath(grab3Score, true);
                    intake.intake(0);
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
                if (pathTimer.getElapsedTime() > 300) {//should be 250, 1000 is for camera
                    pathTimer.resetTimer();
                    intake.SetIntakeMode(Intake.IntakeMode.FEED);

                    setPathState(35);
                }
                break;
            case 35:
                if (pathTimer.getElapsedTime() > 2000) {

                    //trigger.close();
                    //intake.intake(0.9);

                    //move to the pickup 1 position
                    //follower.followPath(scorePickup3, true); //grabPickup1

                    setPathState(40);
                }
                break;
            case 40:
                follower.followPath(scorePark, true); //grabPickup1
                trigger.close();
                intake.intake(0);//stop the intake
                setPathState(-1);
                break;
        }

    }

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());


        //first set of balls
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
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
        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pickup2Grab2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, grab2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), grab2Pose.getHeading())
                .build();
        grab2Score = follower.pathBuilder()
                .addPath(new BezierLine(grab2Pose, scorePose))
                .setLinearHeadingInterpolation(grab2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();
        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
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
}

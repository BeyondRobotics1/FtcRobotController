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

@Autonomous(name = "Red Far Auto", group = "Decode")

public class RedFarAuto extends LinearOpMode {

    //Hardware
    private Shooter shooter;
    private Intake intake;
    //private DriveTrain driveTrain;
    private Trigger trigger;
    private Turret turret;
    private Indexer indexer;
    private Lift lift;
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
    private final Pose startPose = new Pose(55, 136.5, Math.toRadians(-90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(55, 123.5, Math.toRadians(-90)); // Scoring Pose of our robot.
    private final Pose pickup1Pose = new Pose(12.5, 131.75, Math.toRadians(180)); // first pickup spot.
    private final Pose pickup2Pose = new Pose(12.5, 135.25, Math.toRadians(180)); // Second pickup spot
    private final Pose pickup3Pose = new Pose(12.5, 129, Math.toRadians(180)); //Third pickup spot

    private final Pose parkPose = new Pose(55, 112.5, Math.toRadians(-90)); // Where we park

    private Path scorePreload;
    private PathChain pickupScore1, scorePickup1, pickupScore2, scorePickup2, pickupScore3, scorePickup3, scorePark;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Initializing indexer");
        telemetry.update();
        indexer = new Indexer(hardwareMap, this);

        telemetry.addLine("Initializing shooter");
        shooter = new Shooter(hardwareMap, this, DecodeBlackBoard.RED);
        shooter.setShootingLocation(Shooter.ShootingLocation.OUT_ZONE);

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

            telemetry.addLine("Red Far Auto");
            telemetry.addData("Obelisk ID:", tag_id);

            if (tag_id == DecodeBlackBoard.OBELISK_GPP) {
                obelisk_id = tag_id;
                telemetry.addLine("Obelisk: GPP");
            } else if (tag_id == DecodeBlackBoard.OBELISK_PGP) {
                obelisk_id = tag_id;
                telemetry.addLine("Obelisk: PGP");
            } else if (tag_id == DecodeBlackBoard.OBELISK_PPG) {
                obelisk_id = tag_id;
                telemetry.addLine("Obelisk: PPG");
            } else
                telemetry.addLine("Obelisk: Not Detected");

            telemetry.update();
            sleep(100);
        }


        //turret.setServoPosition(Turret.servoPositionAutoShootingRedAlliance);
        turret.setServoPosition(0.150);

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

                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    //Keep the gate open for 1 second
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTime() > 1000) { //500, 650
                    follower.followPath(pickupScore1, true);
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    trigger.open();
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTime() > 100) {//110, 300
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.FEED);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTime() > 950) { //1250

                    trigger.close();
                    intake.intake(0.925);

                    //move to the pickup 1 position
                    follower.followPath(scorePickup2, true); //grabPickup1

                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    //Keep the gate open for 1 second
                    pathTimer.resetTimer();
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTime() > 1000) { //500, 650
                    follower.followPath(pickupScore2, true);
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    trigger.open();
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTime() > 100) {//110, 300
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.FEED);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTime() > 950) { //1250

                    trigger.close();
                    intake.intake(0.925);

                    //move to the pickup 1 position
                    follower.followPath(scorePickup3, true); //grabPickup1

                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    //Keep the gate open for 1 second
                    pathTimer.resetTimer();
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTime() > 1000) { //500, 650
                    follower.followPath(pickupScore3, true);
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    trigger.open();
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTimer.getElapsedTime() > 100) {//110, 300
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.FEED);
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTime() > 950) {
                    follower.followPath(scorePark, true);
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
                    trigger.close();
                    setPathState(19);
                }
                break;
            case 19: //end of auto
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

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();


        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();



        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();
        pickupScore1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();
        pickupScore2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();
        pickupScore3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
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

    void saveAutoState() {
        //pedro pos is in Radian
        Pose p = follower.getPose();
        DecodeBlackBoard.saveAutoEndPose(new Pose2D(DistanceUnit.INCH,
                p.getX(), p.getY(), AngleUnit.DEGREES, Math.toDegrees(p.getHeading())));
    }

    void displayPose() {
        Pose p = follower.getPose();
        telemetry.addData("X", p.getX());
        telemetry.addData("Y", p.getY());
        telemetry.addData("Heading", Math.toDegrees(p.getHeading()));

        DecodeBlackBoard.saveAutoEndPose(new Pose2D(DistanceUnit.INCH,
                p.getX(), p.getY(), AngleUnit.DEGREES, Math.toDegrees(p.getHeading())));

        telemetry.update();
    }
}
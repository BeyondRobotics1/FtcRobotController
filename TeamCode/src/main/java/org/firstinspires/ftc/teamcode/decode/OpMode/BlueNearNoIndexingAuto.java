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
import org.firstinspires.ftc.teamcode.decode.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Trigger;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Disabled
@Autonomous(name = "Blue Near NO Indexing Regional", group = "Decode")
public class BlueNearNoIndexingAuto extends LinearOpMode {

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
    private final Pose startPose = new Pose(30.5, 130.5, Math.toRadians(90)); //31, 131, 90, Start Pose of our robot.
    private final Pose scorePose = new Pose(43, 96, Math.toRadians(136)); // 43, 96, 135// 43, 100 Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    //Highest (First Set)
    private final Pose pickup1Pose = new Pose(41.5, 84, Math.toRadians(180)); //43, 83 Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose grab1Pose = new Pose(17.75, 84, Math.toRadians(180)); //17.5, 83
    private final Pose backout1Pose = new Pose(21, 78, Math.toRadians(180)); //20, 78
    private final Pose openGate1Pose = new Pose(16, 76, Math.toRadians(180)); //16.6, 76

    //Middle (Second Set)
    private final Pose pickup2Pose = new Pose(41.125, 59, Math.toRadians(180)); // 43, 59, Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose grab2Pose = new Pose(9.5, 58, Math.toRadians(180)); //10, 59
    private final Pose backout2Pose = new Pose(15, 58, Math.toRadians(180)); //16.5, 58
    private final Pose openGate2Pose = new Pose(15, 70, Math.toRadians(180)); //15.5, 70 //gate position


    private final Pose openGateSetupPose = new Pose(23, 71, Math.toRadians(180)); //22, 70, 180 Middle (Second Set) backout
    private final Pose openGateStartPose = new Pose(15.8, 69, Math.toRadians(180)); //15.8, 68, 180 //gate position
    private final Pose openGatePose = new Pose(11.5, 63.5, Math.toRadians(150)); //11.5, 63, 150 //gate position

    private final Pose openGatePickupPose = new Pose(11, 59, Math.toRadians(170)); //11, 59, 170
    private final Pose backout22Pose = new Pose(15, 59, Math.toRadians(180)); // 20, 82.5 Middle (Second Set) backout.
    //18, 65.5 //gate position


//    //Middle (Second Set)
//    private final Pose pickup2Pose = new Pose(42.5, 82, Math.toRadians(180)); // 46, 83 Middle (Second Set) picking up start.
//    private final Pose grab2Pose = new Pose(12.25, 82, Math.toRadians(180)); // 12, 82.5 Middle (Second Set) picking up end.
//
//
//
//    private final Pose backout22Pose = new Pose(45, 75.5, Math.toRadians(180)); //42, 75.5

    //Lowest (Third Set)
    private final Pose pickup3Pose = new Pose(42.5, 105, Math.toRadians(180)); //44, 105 Lowest (Third Set) picking up start.
    private final Pose grab3Pose = new Pose(12.25, 105, Math.toRadians(180)); // 12, 105 Highest (First Set) picking up end.

//    //loading zone
//    private final Pose pickup4Pose = new Pose(24, 128, Math.toRadians(135)); //24, 128, 135
//    private final Pose grab4Pose = new Pose(12.25, 132.5, Math.toRadians(180)); //12, 132.5, 180

    //park
    private final Pose parkPose = new Pose(40, 83, Math.toRadians(180)); //40, 80

    private Path scorePreload;
    private PathChain scorePickup1, pickup1Grab1, grab1OpenGate, openGate1Score;
    private PathChain scorePickup2, pickup2Grab2, grab2OpenGate2, openGate2Score;
    private PathChain scoreOpenGate,openGatePickup, gatePickupScore;
    //private PathChain scorePickup3, pickup3Grab3, grab3Score;
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
                DecodeBlackBoard.BLUE_TARGET_POSE,
                DecodeBlackBoard.BLUE,
                false,
                true, true);
        turret.setServoPosition(Turret.servoPositionObeliskDetectionBlueAlliance);
        telemetry.addLine("hardware initialization completed");

        DecodeBlackBoard.saveDefaultAutoEndPose(new Pose2D(DistanceUnit.INCH,
                parkPose.getX(), parkPose.getY(), AngleUnit.DEGREES, Math.toDegrees(parkPose.getHeading())));
        DecodeBlackBoard.saveAutoEndPose(new Pose2D(DistanceUnit.INCH,
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

            telemetry.addLine("Red Near NO Indexing Auto");
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
                if (pathTimer.getElapsedTime() > 70) {//80
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.FEED);

                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > 800) { //900

                    trigger.close();
                    intake.intake(0.95,0.925);

                    //move to the pickup 1 position
                    follower.followPath(scorePickup1, true); //scorePickup2

                    setPathState(11);//11
                }
                break;

            //Highest set of balls
            case 11:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    //grab balls at position 1
                    follower.followPath(pickup1Grab1, true); //grabPickup1
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(grab1OpenGate, true);
                    setPathState(13);
                    pathTimer.resetTimer();
                }
                break;
            case 13:
                if (pathTimer.getElapsedTime() > 1400) {//1500
                    follower.followPath(openGate1Score, true);
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {//should be 110 {
                    pathTimer.resetTimer();
                    trigger.open();
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTime() > 70) {//should be 80
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.FEED);

                    setPathState(17);
                }
                break;
            case 17:
                if (pathTimer.getElapsedTime() > 800) { //shoot balls 900

                    trigger.close();
                    intake.intake(0.95,0.925);

                    //move to the pickup 1 position
                    follower.followPath(scorePickup2, true); //scorePickup2

                    setPathState(21);
                }
                break;


            //Middle set of balls
            case 21:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    //grab balls at position 2
                    follower.followPath(pickup2Grab2, true); //grabPickup1
                    setPathState(22);
                }
                break;
            case 22:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    //move grab1 position to open gate position
                    follower.followPath(grab2OpenGate2, true);
                    setPathState(23);
                }
                break;
            case 23:
                if (pathTimer.getElapsedTime() > 1650) { //1500, takes longer
                    //move from open gate position to score position
                    follower.followPath(openGate2Score, true);
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
                    setPathState(25);
                }
                break;
            case 25:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    trigger.open();
                    setPathState(26);
                }
                break;
            case 26:
                if (pathTimer.getElapsedTime() > 70) {//80
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.FEED);
                    setPathState(27);
                }
                break;
            case 27:
                if (pathTimer.getElapsedTime() > 800) { //shoot balls 800
                    pathTimer.resetTimer();
                    trigger.close();
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);

                    //move to the 3rd spike
                    follower.followPath(scoreOpenGate, true); //grabPickup1

                    setPathState(31);
                }
                break;


            case 31:
                if (pathTimer.getElapsedTime() > 2500) {//Keep gate open 2200
                    pathTimer.resetTimer();

                    //setPathState(-32);

                    //grab balls at gate
                    follower.followPath(openGatePickup, true); //
                    intake.intake(0.95,0.925);
                    setPathState(32);
                }
                break;
            case 32:
                if (pathTimer.getElapsedTime() > 1650) { //1000, in take

                    //setPathState(-33);

                    follower.followPath(gatePickupScore, true);

                    setPathState(33);
                }
                break;
            case 33:
                intake.detectArtifactColors();
                if(intake.detectedArtifacts() == 3)
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);

                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    trigger.open();
                    setPathState(34);
                }
                break;
            case 34:
                if (pathTimer.getElapsedTime() > 70) {//80
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.FEED);

                    setPathState(35);
                }
                break;

            case 35:
                if (pathTimer.getElapsedTime() > 800) { //shoot balls
                    trigger.close();
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);

                    //move to the 3rd spike
                    follower.followPath(scoreOpenGate, true); //grabPickup1

                    setPathState(42);
                }
                break;

            case 42:
                if (pathTimer.getElapsedTime() > 2500) {//Keep gate open 2200
                    pathTimer.resetTimer();

                    //grab balls at gate
                    follower.followPath(openGatePickup, true); //
                    intake.intake(0.95,0.925);
                    setPathState(43);
                }
                break;
            case 43:
                if (pathTimer.getElapsedTime() > 1650) { //1000 in take

                    follower.followPath(gatePickupScore, true);

                    setPathState(44);
                }
                break;
            case 44:
                intake.detectArtifactColors();
                if(intake.detectedArtifacts() == 3)
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);

                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    trigger.open();
                    setPathState(45);
                }
                break;
            case 45:
                if (pathTimer.getElapsedTime() > 70) {//80
                    pathTimer.resetTimer();
                    intake.setIntakeMode(Intake.IntakeMode.FEED);

                    setPathState(46);
                }
                break;

            case 46:
                if (pathTimer.getElapsedTime() > 800) { //shoot balls
                    trigger.close();
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);

                    //move to the 3rd spike
                    follower.followPath(scoreOpenGate, true); //grabPickup1

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

        grab1OpenGate = follower.pathBuilder()
                .addPath(new BezierLine(grab1Pose, backout1Pose))
                .setLinearHeadingInterpolation(grab1Pose.getHeading(), backout1Pose.getHeading())
                .addPath(new BezierLine(backout1Pose, openGate1Pose))
                .setLinearHeadingInterpolation(backout1Pose.getHeading(), openGate1Pose.getHeading())
                .build();

        openGate1Score = follower.pathBuilder()
                .addPath(new BezierLine(openGate1Pose, scorePose))
                .setLinearHeadingInterpolation(openGate1Pose.getHeading(), scorePose.getHeading())
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
        grab2OpenGate2 = follower.pathBuilder()
                .addPath(new BezierLine(grab2Pose, backout2Pose))
                .setLinearHeadingInterpolation(grab2Pose.getHeading(), backout2Pose.getHeading())
                .addPath(new BezierLine(backout2Pose, openGate2Pose))
                .setLinearHeadingInterpolation(backout2Pose.getHeading(), openGate2Pose.getHeading())
                .build();

        openGate2Score = follower.pathBuilder()
                .addPath(new BezierLine(openGate2Pose, scorePose))
                .setLinearHeadingInterpolation(openGate2Pose.getHeading(), scorePose.getHeading())
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


        openGatePickup = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose, openGatePickupPose))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), openGatePickupPose.getHeading())
                .build();
        gatePickupScore = follower.pathBuilder()
                .addPath(new BezierLine(openGatePickupPose, backout22Pose))
                .setLinearHeadingInterpolation(openGatePickupPose.getHeading(), backout22Pose.getHeading())
                .addPath(new BezierLine(backout22Pose, scorePose))
                .setLinearHeadingInterpolation(backout22Pose.getHeading(), scorePose.getHeading())
                .build();


//        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup3Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
//                .build();
//        pickup3Grab3 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup3Pose, grab3Pose))
//                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), grab3Pose.getHeading())
//                .build();
//        grab3Score = follower.pathBuilder()
//                .addPath(new BezierLine(grab3Pose, scorePose))
//                .setLinearHeadingInterpolation(grab3Pose.getHeading(), scorePose.getHeading())
//                .build();

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

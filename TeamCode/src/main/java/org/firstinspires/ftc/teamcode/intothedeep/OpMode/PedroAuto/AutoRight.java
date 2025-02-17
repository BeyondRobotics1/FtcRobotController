package org.firstinspires.ftc.teamcode.intothedeep.OpMode.PedroAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.ClawRotor;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.IntakeSlide;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "Auto Right Specimen", group = "A Into the Deep")

public class AutoRight extends LinearOpMode {

    //log for debugging purpose
    Log log;

    //our robot subsystems
    //Slide slide;
    Claw claw;
    ClawRotor clawRotor;
    OuttakeArm outtakeArm;
    Intake intake;
    IntakeSlide intakeSlide;
    private DigitalChannel touchSensorFrontLimitRight;
    private DigitalChannel touchSensorFrontLimitLeft;

    //Pedro pathing
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    //the state of the auto
    private int pathState;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom right.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(7.25, 62.5, Math.toRadians(0));
    //
    private final Pose pos1 = new Pose(36, 38.5, Math.toRadians(0));

    private final Pose pos11 = new Pose(56, 35, Math.toRadians(0));//36
    private final Pose pos12 = new Pose(56, 28, Math.toRadians(0));
    private final Pose pos13 = new Pose(23, 28, Math.toRadians(0));//24,28


    private final Pose pos21 = new Pose(50, 28, Math.toRadians(0));
    private final Pose pos22 = new Pose(56, 17, Math.toRadians(0));
    private final Pose pos23 = new Pose(23, 17, Math.toRadians(0));//24, 17

    private final Pose pos31 = new Pose(50, 17, Math.toRadians(0));
    private final Pose pos32 = new Pose(56, 9, Math.toRadians(0));
    private final Pose pos33 = new Pose(30, 9, Math.toRadians(0));//20, 9
    private final Pose afterPushingPos = new Pose(31, 9, Math.toRadians(0));

    /** Specimen pickup position from wall */
    private final Pose specimenPickupPos = new Pose(14, 37, Math.toRadians(0)); //12, 36
    private final Pose specimenPickupFinalPos = new Pose(9.5, 37, Math.toRadians(0)); //9, 36

    /** Specimen scoring cycles */
    private final Pose specimenScorePos11 = new Pose(36, 69);//stage to score pose 1
    private final Pose specimenScorePos1 = new Pose(43, 69);//start 69
    private final Pose specimenScorePos2 = new Pose(41.5, 71);//71
    private final Pose specimenScorePos3 = new Pose(41.5, 73);
    private final Pose specimenScorePos4 = new Pose(41.5, 75);
    private final Pose specimenScorePos5 = new Pose(41.5, 77);//78.5

    /** back position for specimen pickup */
    private final Pose specimenPickupPos2 = new Pose(14, 38, Math.toRadians(0)); //12, 38
    private final Pose specimenPickupFinalPos2 = new Pose(9.5, 38, Math.toRadians(0)); //9, 38


    /** Park position */
    private final Pose parkPos = new Pose(16, 24);//15, 24

    /** Paths for pushing and move to pickup position */
    private PathChain push, toSpecimenPickupPosition, toSpecimenPickupFinalPosition;

    /** Paths for scoring */
    private PathChain scoreSpecimen11, scoreSpecimen1, backToSpecimenPickupPosition1;
    private PathChain scoreSpecimen2, backToSpecimenPickupPosition2;
    private PathChain scoreSpecimen3, backToSpecimenPickupPosition3;
    private PathChain scoreSpecimen4, backToSpecimenPickupPosition4;
    private PathChain scoreSpecimen5;

    //shared by scoring to specimen pickup
    private PathChain backToSpecimenPickupFinalPosition;

    /** Path for moving to observation zone for parking */
    private PathChain parking;

    /** Outtake arm takes time to swing down, need to wait this timeout in ms to finish*/
    private final int scoreTimeout = 350;//300
    /** Wait this amount of ms second before moving to the final pickup postion*/
    private final int pickupWaitTime = 100;
    /** wait time for rotating arm to specimen pickup position */
    private final int armResetWaitTime = 800;

    /** Pickup position tolerance in inches*/
    private final double pickupPositionToleranceX = 1.4;//1.25;
    private final double scorePositionToleranceX = 0.75;

    @Override
    public void runOpMode() throws InterruptedException {

        /** Create the log instance with log text file name as IntoTheDeep
         * It should be commented out for competition
         */
        //log = new Log("IntoTheDeep", true);

        /** Create instances of robot subassemblies */
        claw = new Claw(hardwareMap, this);
        clawRotor = new ClawRotor(hardwareMap, this);
        outtakeArm = new OuttakeArm(hardwareMap, this);
        intake = new Intake(hardwareMap, this);
        intakeSlide = new IntakeSlide(hardwareMap);

        touchSensorFrontLimitRight =  hardwareMap.get(DigitalChannel.class, "frontLimitRight");
        touchSensorFrontLimitRight.setMode(DigitalChannel.Mode.INPUT);
        touchSensorFrontLimitLeft =  hardwareMap.get(DigitalChannel.class, "frontLimitLeft");
        touchSensorFrontLimitLeft.setMode(DigitalChannel.Mode.INPUT);

        /** Create Timer instances */
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();

        opmodeTimer.resetTimer();

        /**Create the Pedro Pathing Follower instance*/
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        setPathState(0);

        buildPaths();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("is touch sensor pressed?", isTouchSensorsPressed());
        telemetry.update();

        /** Waiting for Play button being touched */
        waitForStart();

        if (isStopRequested()) return;

        /** Play button is touched, Auto starts */
        intakeSlide.Move(0.48);

        outtakeArm.RotateTo(outtakeArm.SPECIMEN_READY_POSITION);
        claw.open();
        clawRotor.MoveToSpecimenIntakePosition();
        outtakeArm.RotateTo(outtakeArm.SPECIMEN_PICKUP_POSITION);
        intake.MoveToOuttakePosition();

        while (!isStopRequested()  && opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("is touch sensor pressed?", isTouchSensorsPressed());
            telemetry.update();
        }

        //log.close();
    }

    private void autonomousPathUpdate()
    {
        //double headingDelta = 0;
        double poseDeltaX = 0;
        double poseDeltaY = 0;

        switch (pathState) {
            case 0: //push all three samples to observation zone
                follower.followPath(push, true);

                setPathState(1);
                break;
                /** #1 */
            case 1: //move to specimen pickup zone
                poseDeltaX = Math.abs(follower.getPose().getX() - afterPushingPos.getX());
                poseDeltaY = Math.abs(follower.getPose().getY() - afterPushingPos.getY());

                if (poseDeltaX <= 1.2 && poseDeltaY <= 1.2) {
                    follower.followPath(toSpecimenPickupPosition, true);
                    setPathState(2);
                }
                break;
            case 2: //reached the ready position, wait for some time
                //poseDeltaX = Math.abs(follower.getPose().getX() - specimenPickupPos.getX());
                poseDeltaY = Math.abs(follower.getPose().getY() - specimenPickupPos.getY());

                //if the position is reached
                //if (poseDeltaX < 1.2 && poseDeltaY < 1.2) {
                if(poseDeltaY < 1.2) {
                    actionTimer.resetTimer();

                    setPathState(3);
                }
                break;
            case 3: //time out, move to the final specimen pickup position
                if (actionTimer.getElapsedTime() >= pickupWaitTime) {//50

                    follower.followPath(toSpecimenPickupFinalPosition, true);
                    setPathState(4);
                }
                break;
            case 4: //reach the final specimen pickup position & pickup #1
                poseDeltaX = Math.abs(follower.getPose().getX() - specimenPickupFinalPos.getX());
                //poseDeltaY = Math.abs(follower.getPose().getY() - specimenPickupFinalPos.getY());

                //if the position is reached
                if(poseDeltaX <= pickupPositionToleranceX){// && poseDeltaY < 1) {

                    pickupSpecimen();

                    follower.followPath(scoreSpecimen11, true);
                    setPathState(5);//stage position
                }
                break;
            case 5:
                poseDeltaX = Math.abs(follower.getPose().getX() - specimenScorePos11.getX());
                if(poseDeltaX < 1.2)
                {
                    actionTimer.resetTimer();
                    setPathState(6);
                }
                break;
            case 6:
                if(actionTimer.getElapsedTime() >= 50)
                {
                    follower.followPath(scoreSpecimen1, true);
                    setPathState(7);
                }
                break;
            case 7: //score 1 - arm down
                poseDeltaX = Math.abs(follower.getPose().getX() - specimenScorePos1.getX());
                //poseDeltaY = Math.abs(follower.getPose().getY() - specimenScorePos1.getY());

                //logTouchSensor(8);

                //if the position is reached
                if(poseDeltaX <= scorePositionToleranceX || isTouchSensorsPressed()){// && poseDeltaY < 1) {

                    outtakeArm.RotateTo(outtakeArm.SPECIMEN_SCORE_POSITION);
                    actionTimer.resetTimer();

                    setPathState(8);
                }
                break;
            case 8: //score 1 - open claw
                if(actionTimer.getElapsedTime() >= scoreTimeout) {//300

                    releaseSpecimen();

                    actionTimer.resetTimer();
                    follower.followPath(backToSpecimenPickupPosition1, true);

                    setPathState(81);
                }
                break;
            case 81:
                //allow robot moving away from submersible before raising
                //the arm
                if (actionTimer.getElapsedTime() >= armResetWaitTime) {//
                    restoreArm();
                    setPathState(9);
                }
                break;


                /** #2 */
            case 9: //move to pickup pos
                poseDeltaX = Math.abs(follower.getPose().getX() - specimenPickupPos2.getX());
                //poseDeltaY = Math.abs(follower.getPose().getY() - specimenPickupPos2.getY());

                if(poseDeltaX < 1.2) {// && poseDeltaY < 1) {
                    actionTimer.resetTimer();
                    setPathState(10);
                }
                break;
            case 10: //time out, move to the final specimen pickup position
                if (actionTimer.getElapsedTime() >= pickupWaitTime) {//50

                    follower.followPath(backToSpecimenPickupFinalPosition, true);
                    setPathState(11);
                }
                break;
            case 11://final pickup pos reached, pickup #2
                poseDeltaX = Math.abs(follower.getPose().getX() - specimenPickupFinalPos2.getX());
                //poseDeltaY = Math.abs(follower.getPose().getY() - specimenPickupFinalPos2.getY());

                //if the position is reached
                if(poseDeltaX <= pickupPositionToleranceX){// && poseDeltaY < 1) {

                    pickupSpecimen();

                    follower.followPath(scoreSpecimen2, true);
                    setPathState(12);
                }
                break;
            case 12: //score 2 - arm down
                poseDeltaX = Math.abs(follower.getPose().getX() - specimenScorePos2.getX());
                //poseDeltaY = Math.abs(follower.getPose().getY() - specimenScorePos2.getY());

                //logTouchSensor(13);

                //if the position is reached
                if(poseDeltaX <= scorePositionToleranceX || isTouchSensorsPressed()){// && poseDeltaY < 1) {

                    outtakeArm.RotateTo(outtakeArm.SPECIMEN_SCORE_POSITION);
                    actionTimer.resetTimer();

                    setPathState(13);
                }
                break;
            case 13: //score 2 - open claw & move back from specimenScorePos2 to pickup position
                if(actionTimer.getElapsedTime() >= scoreTimeout) {

                    releaseSpecimen();

                    follower.followPath(backToSpecimenPickupPosition2, true);

                    setPathState(141);
                }
                break;
            case 141:
                //allow robot moving away from submersible before raising
                //the arm
                if (actionTimer.getElapsedTime() >= armResetWaitTime) {//
                    restoreArm();
                    setPathState(14);
                }
                break;

                /** #3 */
            case 14: //Reached the pickup position
                poseDeltaX = Math.abs(follower.getPose().getX() - specimenPickupPos2.getX());
                //poseDeltaY = Math.abs(follower.getPose().getY() - specimenPickupPos2.getY());

                if(poseDeltaX < 1.2) {// && poseDeltaY < 1) {
                    actionTimer.resetTimer();
                    setPathState(15);
                }
                break;
            case 15: //move to final pickup position
                if (actionTimer.getElapsedTime() >= pickupWaitTime) {//50

                    follower.followPath(backToSpecimenPickupFinalPosition, true);
                    setPathState(16);
                }
                break;
            case 16://final pickup pos reached, pickup #3
                poseDeltaX = Math.abs(follower.getPose().getX() - specimenPickupFinalPos2.getX());
                //poseDeltaY = Math.abs(follower.getPose().getY() - specimenPickupFinalPos2.getY());

                //if the position is reached
                if(poseDeltaX <= pickupPositionToleranceX){// && poseDeltaY < 1) {

                    pickupSpecimen();

                    follower.followPath(scoreSpecimen3, true);
                    setPathState(17);
                }
                break;
            case 17: //score 3 - arm down
                poseDeltaX = Math.abs(follower.getPose().getX() - specimenScorePos3.getX());
                //poseDeltaY = Math.abs(follower.getPose().getY() - specimenScorePos3.getY());

                //logTouchSensor(18);

                //if the position is reached
                if(poseDeltaX <= scorePositionToleranceX || isTouchSensorsPressed()){// && poseDeltaY < 1) {

                    outtakeArm.RotateTo(outtakeArm.SPECIMEN_SCORE_POSITION);
                    actionTimer.resetTimer();

                    setPathState(18);
                }
                break;
            case 18: //score 3 - open claw
                if(actionTimer.getElapsedTime() >= scoreTimeout) {

                    releaseSpecimen();

                    follower.followPath(backToSpecimenPickupPosition3, true);

                    setPathState(191);
                }
                break;

            case 191:
                //allow robot moving away from submersible before raising
                //the arm
                if (actionTimer.getElapsedTime() >= armResetWaitTime) {//
                    restoreArm();
                    setPathState(19);
                }
                break;

            /** #4 */
            case 19: //Reached the pickup position
                poseDeltaX = Math.abs(follower.getPose().getX() - specimenPickupPos2.getX());
                //poseDeltaY = Math.abs(follower.getPose().getY() - specimenPickupPos2.getY());

                if(poseDeltaX < 1.2) {// && poseDeltaY < 1) {
                    actionTimer.resetTimer();
                    setPathState(20);
                }
                break;
            case 20: //move to final pickup position
                if (actionTimer.getElapsedTime() >= pickupWaitTime) {//50

                    follower.followPath(backToSpecimenPickupFinalPosition, true);
                    setPathState(21);
                }
                break;
            case 21://final pickup pos reached, pickup #3
                poseDeltaX = Math.abs(follower.getPose().getX() - specimenPickupFinalPos2.getX());
                //poseDeltaY = Math.abs(follower.getPose().getY() - specimenPickupFinalPos2.getY());

                //if the position is reached
                if(poseDeltaX <= pickupPositionToleranceX){// && poseDeltaY < 1) {

                    pickupSpecimen();

                    follower.followPath(scoreSpecimen4, true);
                    setPathState(22);
                }
                break;
            case 22: //score 4 - arm down
                poseDeltaX = Math.abs(follower.getPose().getX() - specimenScorePos4.getX());
                //poseDeltaY = Math.abs(follower.getPose().getY() - specimenScorePos4.getY());

                //logTouchSensor(23);

                //if the position is reached
                if(poseDeltaX <= scorePositionToleranceX || isTouchSensorsPressed()){// && poseDeltaY < 1) {

                    outtakeArm.RotateTo(outtakeArm.SPECIMEN_SCORE_POSITION);
                    actionTimer.resetTimer();

                    setPathState(23);
                }
                break;
            case 23: //score 4 - open claw
                if(actionTimer.getElapsedTime() >= scoreTimeout) {

                    releaseSpecimen();

                    follower.followPath(backToSpecimenPickupPosition4, true);

                    setPathState(241);
                }
                break;

            case 241:
                //allow robot moving away from submersible before raising
                //the arm
                if (actionTimer.getElapsedTime() >= armResetWaitTime) {//
                    restoreArm();
                    setPathState(24);
                }
                break;

            /** #5 */
            case 24: //Reached the pickup position
                poseDeltaX = Math.abs(follower.getPose().getX() - specimenPickupPos2.getX());
                //poseDeltaY = Math.abs(follower.getPose().getY() - specimenPickupPos2.getY());

                if(poseDeltaX < 1.2) {// && poseDeltaY < 1) {
                    actionTimer.resetTimer();
                    setPathState(25);
                }
                break;
            case 25: //move to final pickup position
                if (actionTimer.getElapsedTime() >= pickupWaitTime) {//50

                    follower.followPath(backToSpecimenPickupFinalPosition, true);
                    setPathState(26);
                }
                break;
            case 26://final pickup pos reached, pickup #3
                poseDeltaX = Math.abs(follower.getPose().getX() - specimenPickupFinalPos2.getX());
                //poseDeltaY = Math.abs(follower.getPose().getY() - specimenPickupFinalPos2.getY());

                //if the position is reached
                if(poseDeltaX <= pickupPositionToleranceX){// && poseDeltaY < 1) {

                    pickupSpecimen();

                    follower.followPath(scoreSpecimen5, true);
                    setPathState(27);
                }
                break;
            case 27: //score 4 - arm down
                poseDeltaX = Math.abs(follower.getPose().getX() - specimenScorePos5.getX());
                //poseDeltaY = Math.abs(follower.getPose().getY() - specimenScorePos4.getY());

                //logTouchSensor(28);

                //if the position is reached
                if(poseDeltaX <= scorePositionToleranceX || isTouchSensorsPressed()){// && poseDeltaY < 1) {

                    outtakeArm.RotateTo(outtakeArm.SPECIMEN_SCORE_POSITION);
                    actionTimer.resetTimer();

                    setPathState(28);
                }
                break;
            case 28: //score 4 - open claw
                if(actionTimer.getElapsedTime() >= scoreTimeout) {

                    claw.open();
                    sleep(100);

                    follower.followPath(parking, true);

                    setPathState(29);
                }
                break;
            case 29:
                poseDeltaX = Math.abs(follower.getPose().getX() - parkPos.getX());
                //poseDeltaY = Math.abs(follower.getPose().getY() - specimenScorePos4.getY());

                //if the position is reached
                if(poseDeltaX <= 1.2) {// && poseDeltaY < 1) {
                    setPathState(30);
                }
                break;
        }
    }

    private void buildPaths() {

        //pushing path
        push = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(pos1)))
                .setLinearHeadingInterpolation(startPose.getHeading(), pos1.getHeading())
                .addPath(new BezierLine(new Point(pos1), new Point(pos11)))
                .setLinearHeadingInterpolation(pos1.getHeading(), pos11.getHeading())
                .addPath(new BezierLine(new Point(pos11), new Point(pos12)))
                .setLinearHeadingInterpolation(pos11.getHeading(), pos12.getHeading())
                .addPath(new BezierLine(new Point(pos12), new Point(pos13)))
                .setLinearHeadingInterpolation(pos12.getHeading(), pos13.getHeading())
                .addPath(new BezierLine(new Point(pos13), new Point(pos21)))
                .setLinearHeadingInterpolation(pos13.getHeading(), pos21.getHeading())
                .addPath(new BezierLine(new Point(pos21), new Point(pos22)))
                .setLinearHeadingInterpolation(pos21.getHeading(), pos22.getHeading())
                .addPath(new BezierLine(new Point(pos22), new Point(pos23)))
                .setLinearHeadingInterpolation(pos22.getHeading(), pos23.getHeading())
                .addPath(new BezierLine(new Point(pos23), new Point(pos31)))
                .setLinearHeadingInterpolation(pos23.getHeading(), pos31.getHeading())
                .addPath(new BezierLine(new Point(pos31), new Point(pos32)))
                .setLinearHeadingInterpolation(pos31.getHeading(), pos32.getHeading())
                .addPath(new BezierLine(new Point(pos32), new Point(pos33)))
                .setLinearHeadingInterpolation(pos32.getHeading(), pos33.getHeading())
                .addPath(new BezierLine(new Point(pos33), new Point(afterPushingPos)))
                .setLinearHeadingInterpolation(pos33.getHeading(), afterPushingPos.getHeading())
                .build();

        //after pushing
        //move to specimen pickup position
//        toSpecimenPickupPosition = follower.pathBuilder().
//                addPath(new BezierCurve(new Point(pos33), new Point(specimenPickupControlPos), new Point(specimenPickupPos)))
//                .setLinearHeadingInterpolation(pos33.getHeading(), specimenPickupPos.getHeading())
//                .build();

        toSpecimenPickupPosition = follower.pathBuilder().
                addPath(new BezierLine(new Point(afterPushingPos), new Point(specimenPickupPos)))
                .setLinearHeadingInterpolation(afterPushingPos.getHeading(), specimenPickupPos.getHeading())
                .build();

        toSpecimenPickupFinalPosition = follower.pathBuilder().
                addPath(new BezierLine(new Point(specimenPickupPos), new Point(specimenPickupFinalPos)))
                .setLinearHeadingInterpolation(specimenPickupPos.getHeading(), specimenPickupFinalPos.getHeading())
                .build();



        //For the scoring cycles

        //shared by all scoring cycles
        // position specimenPickupFinalPos2-> specimen pickup position
        backToSpecimenPickupFinalPosition = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPickupPos2), new Point(specimenPickupFinalPos2)))
                .setLinearHeadingInterpolation(specimenPickupPos2.getHeading(), specimenPickupFinalPos2.getHeading())
                .build();


        //#1, start with last first pickup position as done after pushing
        scoreSpecimen11 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPickupFinalPos), new Point(specimenScorePos11)))
                .setLinearHeadingInterpolation(specimenPickupFinalPos.getHeading(), specimenScorePos11.getHeading())
                .build();

        scoreSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenScorePos11), new Point(specimenScorePos1)))
                .setLinearHeadingInterpolation(specimenScorePos11.getHeading(), specimenScorePos1.getHeading())
                .build();

        backToSpecimenPickupPosition1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenScorePos1), new Point(specimenPickupPos2)))
                .setLinearHeadingInterpolation(specimenScorePos1.getHeading(), specimenPickupPos2.getHeading())
                .build();


        //#2
        scoreSpecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPickupFinalPos2), new Point(specimenScorePos2)))
                .setLinearHeadingInterpolation(specimenPickupFinalPos2.getHeading(), specimenScorePos2.getHeading())
                .build();
        backToSpecimenPickupPosition2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenScorePos2), new Point(specimenPickupPos2)))
                .setLinearHeadingInterpolation(specimenScorePos2.getHeading(), specimenPickupPos2.getHeading())
                .build();


        scoreSpecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPickupFinalPos2), new Point(specimenScorePos3)))
                .setLinearHeadingInterpolation(specimenPickupFinalPos2.getHeading(), specimenScorePos3.getHeading())
                .build();
        backToSpecimenPickupPosition3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenScorePos3), new Point(specimenPickupPos2)))
                .setLinearHeadingInterpolation(specimenScorePos3.getHeading(), specimenPickupPos2.getHeading())
                .build();


        scoreSpecimen4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPickupFinalPos2), new Point(specimenScorePos4)))
                .setLinearHeadingInterpolation(specimenPickupFinalPos2.getHeading(), specimenScorePos4.getHeading())
                .build();
        backToSpecimenPickupPosition4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenScorePos4), new Point(specimenPickupPos2)))
                .setLinearHeadingInterpolation(specimenScorePos4.getHeading(), specimenPickupPos2.getHeading())
                .build();

        scoreSpecimen5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPickupFinalPos2), new Point(specimenScorePos5)))
                .setLinearHeadingInterpolation(specimenPickupFinalPos2.getHeading(), specimenScorePos5.getHeading())
                .build();
        parking =  follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenScorePos5), new Point(parkPos)))
                .setLinearHeadingInterpolation(specimenScorePos5.getHeading(), parkPos.getHeading())
                .build();
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    private void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }

    private void pickupSpecimen()
    {
        claw.close();
        sleep(100);
        outtakeArm.RotateTo(outtakeArm.SPECIMEN_READY_POSITION);
        clawRotor.MoveToSampleIntakePosition();
    }

    private void releaseSpecimen()
    {
        claw.open();
        sleep(100);
        //clawRotor.MoveToSpecimenIntakePosition();
        //outtakeArm.RotateTo(outtakeArm.SPECIMEN_PICKUP_POSITION);
    }

    private void restoreArm()
    {
        clawRotor.MoveToSpecimenIntakePosition();
        outtakeArm.RotateTo(outtakeArm.SPECIMEN_PICKUP_POSITION);
    }

    private void logXYDelta(Pose currentPose, Pose targetPose)
    {
        if(log != null) {
            double poseDeltaX = currentPose.getX() - targetPose.getX();
            double poseDeltaY = currentPose.getY() - targetPose.getY();

            String msg = "C: (" + currentPose.getX() + ", " + currentPose.getY() +
                    "), T: (" + targetPose.getX() + ", " + targetPose.getY() +
                    ")" + "Delat : (" + poseDeltaX + ", " + poseDeltaY + ")";

            log.addData(msg);
            log.update();
        }
    }

    private boolean isTouchSensorsPressed(){
        boolean rightSensorPressed = !touchSensorFrontLimitRight.getState();
        boolean leftSensorPressed = !touchSensorFrontLimitLeft.getState();

        if(rightSensorPressed || leftSensorPressed){
            return true;
        }
        else {
            return false;
        }
    }

    private void logTouchSensor(int step)
    {
        if(log != null) {
            String msg = "Step: " + step + "Left Pressed -  " + !touchSensorFrontLimitLeft.getState() +
                    ", Right Pressed - " + !touchSensorFrontLimitRight.getState();

            log.addData(msg);
            log.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.intothedeep.OpMode.PedroAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Helper;
import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.ClawRotor;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.IntakeSlide;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "Blue Right Intake (IntoTheDeep)", group = "A Into the Deep")

public class BlueRightIntake extends LinearOpMode {

    //our robot subsystems
    //Slide slide;
    Claw claw;
    ClawRotor clawRotor;
    OuttakeArm outtakeArm;
    Intake intake;
    IntakeSlide intakeSlide;

    //Pedro pathing
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    //the state of the auto
    private int pathState;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(7.25, 62.5, Math.toRadians(0));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(30, 37, Math.toRadians(-45));
    private final Pose drop1Pose = new Pose(26.25, 36.75, Math.toRadians(-109));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(29.25, 26.75, Math.toRadians(-45));
    private final Pose drop2Pose = new Pose(27.25, 24.75, Math.toRadians(-109));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0));
    private final Pose drop3Pose = new Pose(37, 121, Math.toRadians(0));

    /** Specimen pickup position from wall */
    private final Pose specimenPickupPos = new Pose(12, 36, Math.toRadians(0));

    private PathChain pickUp1, drop1, pickUp2, drop2, pickUp3, Drop3;
    private Path specimenPickup;


    Log log;
    @Override
    public void runOpMode() throws InterruptedException {

        log = new Log("IntoTheDeep", true);

        claw = new Claw(hardwareMap, this);
        clawRotor = new ClawRotor(hardwareMap, this);
        //clawRotor.SetClawDown();

        outtakeArm = new OuttakeArm(hardwareMap, this);
        intake = new Intake(hardwareMap, this);
        intakeSlide = new IntakeSlide(hardwareMap);
        intakeSlide.Move(0.49);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);


        setPathState(0);

        buildPaths();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()  && opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }

        log.close();
    }

    private void autonomousPathUpdate()
    {
        double headingDelta = 0;
        double poseDeltaX = 0;
        double poseDeltaY = 0;

        switch (pathState) {
            case 0:
                follower.followPath(pickUp1, true);

                outtakeArm.Rotate(outtakeArm.SPECIMEN_READY_POSITION);
                claw.open();
                clawRotor.SetClawUp();
                outtakeArm.Rotate(outtakeArm.SPECIMEN_PICKUP_POSITION);

                intake.MoveToIntakePosition();
                setPathState(1);
                break;
            case 1: //move to #1 pickup position & pickup it up
                headingDelta = calculateHeadingDelta(follower.getPose(), pickup1Pose);

                //if the angle is aligned, less than 2 degrees
                if(Math.abs(headingDelta) <= 2)
                {
                    intake.SetIntakeSpinner(Intake.IntakeMode.IN);
                    intakeSlide.Move(0.65);
                    actionTimer.resetTimer();

                    setPathState(2);
                }
                break;
            case 2: //move to #1 drop position
                if(actionTimer.getElapsedTime() >= 800) {

                    intake.MoveToOuttakePosition();

                    follower.followPath(drop1, true);

                    setPathState(3);
                }
                break;
            case 3://drop the #1
                headingDelta = calculateHeadingDelta(follower.getPose(), drop1Pose);

                if(Math.abs(headingDelta) <= 2)
                {
                    //head down and drop out the sample
                    intake.SetIntakeSpinner(Intake.IntakeMode.OUT);
                    intake.MoveToHeadDownPosition();

                    actionTimer.resetTimer();

                    setPathState(8);
                }
                break;
            case 4://slide back and move to #2 pickup position
                if(actionTimer.getElapsedTime() >= 400) {
                    //pull back slide
                    intakeSlide.Move(0.45);

                    follower.followPath(pickUp2, true);
                    setPathState(5);
                }
                break;
            case 5: //pickup #2
                headingDelta = calculateHeadingDelta(follower.getPose(), pickup2Pose);

                //poseDeltaX = Math.abs(follower.getPose().getX() - pickup2Pose.getX());
                //poseDeltaY = Math.abs(follower.getPose().getY() - pickup2Pose.getY());

                //if the angle is aligned, less than 2 degrees
                if(Math.abs(headingDelta) <= 2)
                {
                    intake.MoveToIntakePosition();
                    intake.SetIntakeSpinner(Intake.IntakeMode.IN);

                    intakeSlide.Move(0.65);
                    actionTimer.resetTimer();
                    setPathState(6);
                }
                break;
            case 6: //move to #2 drop postion
                if(actionTimer.getElapsedTime() >= 800) {

                    intake.MoveToOuttakePosition();
                    follower.followPath(drop2, true);

                    setPathState(7);
                }
                break;
            case 7: //drop #2
                headingDelta = calculateHeadingDelta(follower.getPose(), drop2Pose);

                if(Math.abs(headingDelta) <= 2)
                {
//                    //head down and drop out the sample
//                    intake.MoveToHeadDownPosition();
//                    intake.SetIntakeSpinner(Intake.IntakeMode.OUT);
//
//                    sleep(400);
//
//                    //pull back slide
//                    intakeSlide.Move(0.35);

                    follower.followPath(pickUp2, true);

                    setPathState(8);
                }
                break;
            case 8:
                break;
        }
    }

    private void buildPaths() {
        pickUp1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup1Pose.getHeading())
                .build();

        drop1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(drop1Pose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), drop1Pose.getHeading())
                .build();

        pickUp2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(drop1Pose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(drop1Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        drop2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(drop2Pose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), drop2Pose.getHeading())
                .build();



    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    private void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
        //actionTimer.resetTimer();
    }

    private double calculateHeadingDelta(Pose currentPose, Pose targetPose)
    {
        double headingDelta = currentPose.getHeading() - targetPose.getHeading();
        headingDelta = Math.toDegrees(Helper.normDelta(headingDelta));

//        String msg = "C: " + Math.toDegrees(follower.getPose().getHeading()) +", T: " +
//                Math.toDegrees(pickup1Pose.getHeading()) + ", Delat: " + headingDelta;
//
//        log.addData(msg);
//        log.update();

        return headingDelta;
    }
}





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
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "Blue Right Push (IntoTheDeep)", group = "A Into the Deep")
@Disabled
public class BlueRightPush extends LinearOpMode {

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
    //
    private final Pose pos1 = new Pose(36, 38.5, Math.toRadians(0));

    private final Pose pos11 = new Pose(56, 36, Math.toRadians(0));
    private final Pose pos12 = new Pose(56, 28, Math.toRadians(0));
    private final Pose pos13 = new Pose(24, 28, Math.toRadians(0));


    private final Pose pos21 = new Pose(50, 28, Math.toRadians(0));
    private final Pose pos22 = new Pose(56, 17, Math.toRadians(0));
    private final Pose pos23 = new Pose(24, 17, Math.toRadians(0));

    private final Pose pos31 = new Pose(50, 17, Math.toRadians(0));
    private final Pose pos32 = new Pose(56, 10, Math.toRadians(0));
    private final Pose pos33 = new Pose(20, 10, Math.toRadians(0));


    private PathChain push;

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

        intakeSlide.Move(0.48);

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
                follower.followPath(push, true);

                setPathState(1);
                break;
            case 1:
                //headingDelta = follower.getPose().getHeading() - pos33.getHeading();
                //headingDelta = Math.toDegrees(Helper.normDelta(headingDelta));
                //String msg = "C: " + Math.toDegrees(follower.getPose().getHeading()) +", T: " +
                //        Math.toDegrees(pickup1Pose.getHeading()) + ", Delat: " + headingDelta;

                //log.addData(msg);
                //log.update();

                poseDeltaX = Math.abs(follower.getPose().getX() - pos33.getX());
                poseDeltaY = Math.abs(follower.getPose().getY() - pos33.getY());

                //if the angle is aligned, less than 2 degrees
                if(Math.abs(poseDeltaX) <1 && Math.abs(poseDeltaY) < 1)
                {
                    setPathState(2);
                }
                break;
            case 2:
                break;
        }
    }

    private void buildPaths() {
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
                .build();
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    private void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
        //actionTimer.resetTimer();
    }
}

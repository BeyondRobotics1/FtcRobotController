package org.firstinspires.ftc.teamcode.biobuzz.opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.ShooterGroup;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.TriggerGroup;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "BioBuzz Auto BLUE AUDIENCE", group = "BioBuzz")
public class AutoBlueAudience extends LinearOpMode {

    private Intake intake;
    private ShooterGroup shooters;
    private TriggerGroup triggers;
    private Turret turret;

    Follower follower;

    //status
    boolean shooterStopped = true;
    private Timer pathTimer;
    private int pathState = 0;

    private final Pose startPose = new Pose(BioBuzzDashboard.BLUE_AUDIENCE_AUTO_START_POSE.getX(DistanceUnit.INCH),
            BioBuzzDashboard.BLUE_AUDIENCE_AUTO_START_POSE.getY(DistanceUnit.INCH),
            BioBuzzDashboard.BLUE_AUDIENCE_AUTO_START_POSE.getHeading(AngleUnit.RADIANS));

    private final Pose controlScoreToParkPose = new Pose(BioBuzzDashboard.RED_SCORE_AUTO_START_POSE.getX(DistanceUnit.INCH),
            119, BioBuzzDashboard.RED_SCORE_AUTO_START_POSE.getHeading(AngleUnit.RADIANS));

    private final Pose parkPose = new Pose(BioBuzzDashboard.BLUE_AUDIENCE_AUTO_PARK_POSE.getX(DistanceUnit.INCH),
            BioBuzzDashboard.BLUE_AUDIENCE_AUTO_PARK_POSE.getY(DistanceUnit.INCH),
            BioBuzzDashboard.BLUE_AUDIENCE_AUTO_PARK_POSE.getHeading(AngleUnit.RADIANS));

    private PathChain startPark;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();

        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        List<LynxModule> hubs = initBulkCaching();

        while (!isStarted() && !isStopRequested()) {

            follower.update();
            telemetry.addLine("Auto BLUE AUDIENCE selected");
            telemetry.addLine("Auto BLUE AUDIENCE selected");
            displayPose();
        }

        pathTimer.resetTimer();
        setPathState(0);

        while (!isStopRequested() && opModeIsActive()) {
            hubs.forEach(LynxModule::clearBulkCache);

            follower.update();

            autonomousPathUpdate();

            if(!shooterStopped)
                shooters.doFlyWheelPID();

            saveAutoState();

            displayPose();
        }
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (pathTimer.getElapsedTime() > 15000) {//wait 2 seconds to start and stabilize the shooter
                    shooters.setPower(Shooter.kTargetSpeedMediumPollen);
                    shooterStopped = false;

                    pathTimer.resetTimer();

                    setPathState(1);
                }
                break;
            case 1:
                if (pathTimer.getElapsedTime() > 5000) { //wait 1 more seconds to shoot

                    intake.setIntakeMode(Intake.IntakeMode.FEED);
                    triggers.open();

                    pathTimer.resetTimer();

                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 2000) //wait for 2 second to complete shooting
                {
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
                    triggers.close();

                    pathTimer.resetTimer();

                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > 1000) { //try shoot if stuck
                    intake.setIntakeMode(Intake.IntakeMode.FEED);
                    triggers.open();

                    pathTimer.resetTimer();

                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTime() > 1000)
                {
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
                    triggers.close();
                    shooters.stop();

                    shooterStopped = true;

                    follower.followPath(startPark, true); //grab garden

                    setPathState(100);
                }
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
        startPark = follower.pathBuilder()
                .addPath(new BezierLine(startPose, controlScoreToParkPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), controlScoreToParkPose.getHeading())
                .addPath(new BezierLine(controlScoreToParkPose, parkPose))
                .setLinearHeadingInterpolation(controlScoreToParkPose.getHeading(), parkPose.getHeading())
                .build();
    }

    private List<LynxModule> initBulkCaching() {
        telemetry.addLine("Initializing LynxModule");
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry.addLine("LynxModule initialized");
        telemetry.update();
        return hubs;
    }

    private void warmUpShooter() throws InterruptedException {
        shooters.setPower(0.9);
        sleep(150);

        for (int i = 0; i < 30 && !isStopRequested(); i++) {
            shooters.doFlyWheelPID();
            sleep(10);
        }
    }

    private void initSubsystems() {
        telemetry.addLine("Initializing shooters");
        telemetry.update();
        shooters = new ShooterGroup(hardwareMap, this);

        telemetry.addLine("Initializing intake");
        telemetry.update();
        intake = new Intake(hardwareMap, this);

        telemetry.addLine("Initializing triggers");
        telemetry.update();
        triggers = new TriggerGroup(hardwareMap);
        triggers.close();

        telemetry.addLine("Initializing turret");
        telemetry.update();
        turret = new Turret(hardwareMap, this, BioBuzzDashboard.BLUE_AUDIENCE_AUTO_START_POSE,
                BioBuzzDashboard.ALLIANCE_BLUE, false, false);
        turret.setServoPosition(Turret.BLUE_AUDIENCE_AUTO_SERVO_POSITION);
    }

    void setPathState(int newPathState) {
        this.pathState = newPathState;
    }

    void saveAutoState()
    {
        //pedro pos is in Radian
        Pose p = follower.getPose();
        BioBuzzDashboard.saveAutoEndPose(blackboard, new Pose2D(DistanceUnit.INCH,
                p.getX(), p.getY(), AngleUnit.DEGREES, Math.toDegrees(p.getHeading())));
    }

    void displayPose()
    {
        Pose p = follower.getPose();
        telemetry.addData("X", p.getX());
        telemetry.addData("Y", p.getY());
        telemetry.addData("Heading",  Math.toDegrees(p.getHeading()));

        BioBuzzDashboard.saveAutoEndPose(blackboard, new Pose2D(DistanceUnit.INCH,
                p.getX(), p.getY(), AngleUnit.DEGREES, Math.toDegrees(p.getHeading())));

        telemetry.update();
    }


}


package org.firstinspires.ftc.teamcode.biobuzz.opmode;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.ShooterGroup;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.TriggerGroup;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Turret;

import java.util.List;

/**
 * Shared implementation for the red and blue BioBuzz driver-controlled OpModes.
 *
 * <p>The concrete OpModes only provide alliance-specific poses and labels so that
 * FTC can continue to expose separate RED and BLUE entries on the Driver Station.</p>
 */
public abstract class BioBuzzTeleOpBase extends LinearOpMode {
    protected enum StartingSide {
        AUDIENCE,
        SCORE
    }

    private DriveTrain driveTrain;
    private Intake intake;
    private ShooterGroup shooters;
    private TriggerGroup triggers;
    private Turret turret;

    private Timer gameTimer;
    private Gamepad.RumbleEffect strongRumbleEffect;

    private boolean isAudienceSideTeleOp = true;
    private boolean isIntakeOn;
    private boolean isShooterOn;
    private boolean enableAutoAiming = true;
    private boolean enableAutoShootingSpeed = true;
    private boolean isHeadingToGoal;
    private int rumbleEndgame;

    private Pose2D robotPose;
    private ShootAutoCompleteState shootAutoCompleteMode = ShootAutoCompleteState.COMPLETED;

    protected abstract int alliance();

    protected abstract String allianceName();

    protected abstract Pose2D audienceParkPose();

    protected abstract Pose2D scoreParkPose();

    protected abstract Pose2D imuResetPose();

    /**
     * Returns a fixed side for dedicated OpModes, or {@code null} when the
     * driver should choose the side during initialization.
     */
    protected StartingSide fixedStartingSide() {
        return null;
    }

    @Override
    public final void runOpMode() throws InterruptedException {
        initSubsystems();
        List<LynxModule> hubs = initBulkCaching();

        isIntakeOn = false;
        isShooterOn = true;

        prepareStartingSide();
        if (isStopRequested()) {
            return;
        }

        robotPose = validAutoEndPoseOrDefault();
        turret = new Turret(hardwareMap, this, robotPose, alliance(), false, false);
        turret.resetTurretHeading();

        gameTimer.resetTimer();
        warmUpShooter();
        isIntakeOn = true;

        while (opModeIsActive() && !isStopRequested()) {
            hubs.forEach(LynxModule::clearBulkCache);

            telemetry.addLine(selectedModeLabel());
            telemetry.addLine("");

            driveTrain.setPower(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x);

            intakeOp();
            turretOp();
            shootOp();
            doRumble();

            telemetry.update();
        }
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

    private void prepareStartingSide() {
        StartingSide fixedSide = fixedStartingSide();
        if (fixedSide != null) {
            isAudienceSideTeleOp = fixedSide == StartingSide.AUDIENCE;
            waitForStartWithFixedSide();
            return;
        }

        selectStartingSide();
    }

    private void waitForStartWithFixedSide() {
        while (!isStarted() && !isStopRequested()) {
            robotPose = BioBuzzDashboard.robotAutoEndPose(blackboard);
            telemetry.addLine(selectedModeLabel());
            addAutoEndPoseTelemetry();
            telemetry.update();
        }
    }

    private void selectStartingSide() {
        while (!isStarted() && !isStopRequested()) {
            robotPose = BioBuzzDashboard.robotAutoEndPose(blackboard);

            telemetry.addLine(selectedModeLabel());
            telemetry.addLine(selectedModeLabel());
            telemetry.addLine("");

            addAutoEndPoseTelemetry();
            telemetry.update();
        }
    }

    private void addAutoEndPoseTelemetry() {
        telemetry.addData("Auto end X (Inch):", robotPose.getX(DistanceUnit.INCH));
        telemetry.addData("Auto end Y (Inch):", robotPose.getY(DistanceUnit.INCH));
        telemetry.addData(
                "Auto end Heading (Degree):",
                robotPose.getHeading(AngleUnit.DEGREES));
    }

    private Pose2D validAutoEndPoseOrDefault() {
        if (robotPose != null && robotPose.getX(DistanceUnit.INCH) >= 10.0) {
            return robotPose;
        }
        return isAudienceSideTeleOp ? audienceParkPose() : scoreParkPose();
    }

    private String selectedModeLabel() {
        return "TeleOp " + allianceName() + " "
                + (isAudienceSideTeleOp ? "AUDIENCE" : "SCORE")
                + " selected";
    }

    private void warmUpShooter() throws InterruptedException {
        shooters.setPower(0.9);
        sleep(150);

        for (int i = 0; i < 30 && !isStopRequested(); i++) {
            shooters.doFlyWheelPID();
            sleep(10);
        }
    }

    private void intakeOp() {
        if (gamepad1.rightBumperWasPressed()) {
            isIntakeOn = !isIntakeOn;
        }

        if (Math.abs(gamepad1.right_trigger) > 0.5) {
            intake.setIntakeMode(Intake.IntakeMode.OUT);
        } else if (!isIntakeOn) {
            intake.setIntakeMode(Intake.IntakeMode.IDLE);
        } else if (shootAutoCompleteMode == ShootAutoCompleteState.COMPLETED) {
            intake.setIntakeMode(Intake.IntakeMode.IN);
        } else {
            intake.setIntakeMode(Intake.IntakeMode.FEED);
        }

        if (gamepad1.leftBumperWasPressed()) {
            shootAutoCompleteMode = ShootAutoCompleteState.START;
            triggers.open();
        } else if (gamepad1.leftBumperWasReleased()) {
            shootAutoCompleteMode = ShootAutoCompleteState.COMPLETED;
            triggers.close();
        }
    }

    private void turretOp() {
        isHeadingToGoal = turret.isHeadingToGoal();

        //reset IMU using gamepad2.left_bumper
        if (gamepad2.left_bumper) {
            turret.setIMUPose(imuResetPose());
        }

        //toggle auto aiming gamepad2.right_bumper
        if (gamepad2.rightBumperWasPressed()) {
            enableAutoAiming = !enableAutoAiming;
            if (!enableAutoAiming) {
                turret.resetTurretHeading();
            }
        }

        // Enable when turret auto-aiming is ready for driver use.
        // turret.autoAim(enableAutoAiming);
    }

    private void shootOp() {
        if (gamepad2.dpadUpWasPressed()) {
            enableAutoShootingSpeed = true;
        }
        if (gamepad2.dpadDownWasPressed()) {
            enableAutoShootingSpeed = false;
        }

        // MEDIUM is currently the only enabled driver-controlled speed. Keeping
        // this decision here gives both alliances identical shooter behavior.
        if (!enableAutoShootingSpeed
                || (intake.detectedArtifacts() > 0 && isHeadingToGoal)) {
            shooters.setShootingLocation(Shooter.ShootingLocation.MEDIUM);
        }

        if (isShooterOn) {
            shooters.doFlyWheelPID();
        } else {
            shooters.stop();
        }
    }

    private void doRumble() {
        if (gameTimer.getElapsedTimeSeconds() >= 80 && rumbleEndgame == 0) {
            rumbleEndgame = 1;
            gamepad1.runRumbleEffect(strongRumbleEffect);
            gamepad2.runRumbleEffect(strongRumbleEffect);
        }

        if (gameTimer.getElapsedTimeSeconds() >= 100 && rumbleEndgame == 1) {
            rumbleEndgame = 2;
            gamepad1.runRumbleEffect(strongRumbleEffect);
            gamepad2.runRumbleEffect(strongRumbleEffect);
        }
    }

    private void initSubsystems() {
        telemetry.addLine("Initializing drive train");
        telemetry.update();
        driveTrain = new DriveTrain(hardwareMap, this, false);

        telemetry.addLine("Initializing shooters");
        telemetry.update();
        shooters = new ShooterGroup(hardwareMap, this);

        telemetry.addLine("Initializing intake");
        telemetry.update();
        intake = new Intake(hardwareMap, this);

        telemetry.addLine("Initializing triggers");
        telemetry.update();
        triggers = new TriggerGroup(hardwareMap);

        gameTimer = new Timer();
        strongRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.5, 100)
                .addStep(1, 1, 350)
                .addStep(0.5, 0.5, 100)
                .addStep(1, 1, 350)
                .addStep(0.5, 0.5, 100)
                .build();
    }
}

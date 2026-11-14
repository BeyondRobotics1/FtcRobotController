package org.firstinspires.ftc.teamcode.biobuzz.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.biobuzz.opmode.ShootAutoCompleteState;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.TriggerGroup;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.VelocityPIDFShooter;

import java.util.List;

@TeleOp(name = "Pollen Velocity PIDF Shooter Test", group = "BioBuzz Test")
public class VeclocityPIDFShooterTest extends LinearOpMode {
    private Intake intake;
    private VelocityPIDFShooter shooter;
    private TriggerGroup triggers;

    private boolean isIntakeOn;
    private ShootAutoCompleteState shootAutoCompleteMode = ShootAutoCompleteState.COMPLETED;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new VelocityPIDFShooter(
                hardwareMap,
                this,
                "pollenFlywheel",
                Shooter.kTargetSpeedFarFarPollen,
                Shooter.kTargetSpeedFarPollen,
                Shooter.kTargetSpeedMediumPollen,
                Shooter.kTargetSpeedNearPollen);

        telemetry.addLine("Initializing intake");
        telemetry.update();
        intake = new Intake(hardwareMap, this);

        telemetry.addLine("Initializing triggers");
        telemetry.update();
        triggers = new TriggerGroup(hardwareMap);

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        PIDFCoefficients pidf = shooter.getPIDParameters();
        telemetry.addData("Hub velocity PIDF", "P %.4f  I %.4f  D %.4f  F %.4f",
                pidf.p, pidf.i, pidf.d, pidf.f);
        telemetry.addLine("Right bumper: toggle intake");
        telemetry.addLine("Right trigger: reverse intake");
        telemetry.addLine("Left bumper: open triggers");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        warmUpShooter();

        while (opModeIsActive()) {
            hubs.forEach(LynxModule::clearBulkCache);

            intakeOp();
            shooter.doFlyWheelPID();

            telemetry.addData("Flywheel current (A)", "%.2f", shooter.getCurrentAmps());
            telemetry.addData("Flywheel reported power", "%.2f", shooter.getAppliedPower());
            telemetry.update();
        }

        shooter.stop();
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

    private void warmUpShooter() throws InterruptedException {
        final int warmupSteps = 8;
        final double warmupSpeed = 0.65;

        for (int step = 1; step <= warmupSteps && !isStopRequested(); step++) {
            shooter.setPower(warmupSpeed * step / warmupSteps);
            sleep(25);
        }

        for (int i = 0; i < 30 && !isStopRequested(); i++) {
            shooter.doFlyWheelPID();
            sleep(10);
        }
    }
}

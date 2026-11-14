package org.firstinspires.ftc.teamcode.biobuzz.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.opmode.ShootAutoCompleteState;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.TriggerGroup;

import java.util.List;

//@Disabled


@TeleOp(name = "Pollen Shooter Test", group = "BioBuzz Test")

public class ShooterTest extends LinearOpMode {

    private Intake intake;
    Shooter shooter;
    TriggerGroup triggers;

    boolean isIntakeOn = false;
    ShootAutoCompleteState shootAutoCompleteMode = ShootAutoCompleteState.COMPLETED;

    @Override
    public void runOpMode() throws InterruptedException {

        shooter = new Shooter(hardwareMap,
                this,
                "pollenFlywheel",
                Shooter.kPPollen,
                Shooter.kIPollen,
                Shooter.kDPollen,
                Shooter.kFPollen,
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

        telemetry.update();

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

        if (isStopRequested()) return;

        warmUpShooter();

        while (!isStopRequested() && opModeIsActive()) {

            hubs.forEach(LynxModule::clearBulkCache);

            intakeOp();
            shooter.doFlyWheelPID();

            telemetry.update();
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

    private void warmUpShooter() throws InterruptedException {
        final int warmupSteps = 8;
        final double warmupPower = 0.65;
        for (int step = 1; step <= warmupSteps && !isStopRequested(); step++) {
            shooter.setPower(warmupPower * step / warmupSteps);
            sleep(25);
        }

        for (int i = 0; i < 30 && !isStopRequested(); i++) {
            shooter.doFlyWheelPID();
            sleep(10);
        }
    }
}

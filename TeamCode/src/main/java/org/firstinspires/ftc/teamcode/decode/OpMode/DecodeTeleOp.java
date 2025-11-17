package org.firstinspires.ftc.teamcode.decode.OpMode;

import android.graphics.Color;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriveTrain;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Trigger;

import java.util.List;

@TeleOp(name = "Decode TeleOp", group = "A")


public class DecodeTeleOp extends LinearOpMode {

    enum ShootAutoCompleteMode
    {
        START,
        OPEN_TRIGGER,
        TRIGGER_OPENED,
        STOP,
        COMPLETED
    }

    private Timer actionTimer;
    private Timer gameTimer;

    private Shooter shooter;
    private Intake intake;
    private DriveTrain driveTrain;
    private Trigger trigger;

    private boolean isIntakOn;
    private boolean isShooterOn;
    private int shootingSpeed; //1 slow, 2 middle, 3 fast

    ShootAutoCompleteMode shootAutoCompleteMode;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing drive train");
        telemetry.update();
        driveTrain = new DriveTrain(hardwareMap, this, false);

        telemetry.addLine("Initializing shooter");
        shooter = new Shooter(hardwareMap, this);

        telemetry.addLine("Initializing intake");
        intake = new Intake(hardwareMap, this);

        telemetry.addLine("Initializing trigger");
        trigger = new Trigger(hardwareMap, this);
        trigger.close();

        actionTimer = new Timer();
        gameTimer = new Timer();

        shootAutoCompleteMode = ShootAutoCompleteMode.STOP;

        telemetry.addLine("hardware initialization completed");


        telemetry.addLine("initializing LynxModule");
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry.addLine("LynxModule initialized");


        isIntakOn = false;
        isShooterOn = true;

        waitForStart();

        if(isStopRequested()) return;

        shooter.shoot();

        while(!isStopRequested() && opModeIsActive())
        {
            hubs.forEach(LynxModule::clearBulkCache);


            intakeOp();
            shootOp();


            driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }

    private void intakeOp()
    {
        //use left bumper button to toggle
        //intake
        if(gamepad1.leftBumperWasPressed())
            isIntakOn = !isIntakOn;

        //intake control
        if (gamepad1.right_bumper) //right bumper take in
            intake.SetIntakeMode(Intake.IntakeMode.OUT);
        else {
            if (isIntakOn) { //intake mode
                intake.intake(-1);
            }
            else
                intake.intake(0);
        }

        if (gamepad1.aWasPressed())
            shootAutoCompleteMode = ShootAutoCompleteMode.START;
        else if (gamepad1.aWasReleased()) {
            shootAutoCompleteMode = ShootAutoCompleteMode.COMPLETED;
            trigger.close();
        }

        if(shootAutoCompleteMode == ShootAutoCompleteMode.START ||
            shootAutoCompleteMode == ShootAutoCompleteMode.COMPLETED)
        {
            switch (shootAutoCompleteMode) {
                case START:
                    actionTimer.resetTimer();
                    shootAutoCompleteMode = ShootAutoCompleteMode.OPEN_TRIGGER;
                    trigger.open();
                    break;
                case OPEN_TRIGGER:
                    if (actionTimer.getElapsedTime() > 250) {
                        shootAutoCompleteMode = ShootAutoCompleteMode.TRIGGER_OPENED;
                    }
                case TRIGGER_OPENED:
                    actionTimer.resetTimer();
                    intake.SetIntakeMode(Intake.IntakeMode.FEED);
                    shootAutoCompleteMode = ShootAutoCompleteMode.STOP;
                    break;
                case STOP:
                    if (actionTimer.getElapsedTime() > 1000) {
                        actionTimer.resetTimer();
                        trigger.close();
                        shootAutoCompleteMode = ShootAutoCompleteMode.COMPLETED;
                    }
                    break;
                case COMPLETED:
                    trigger.close();
                    break;
            }
        }
    }

    private void shootOp()
    {
        //use gamepad1 X button to toggle
        //shooter motors
        if(gamepad1.xWasPressed())
            isShooterOn = !isShooterOn;

        if(isShooterOn)
            shooter.shoot();
        else
            shooter.stop();
    }

    private void turretOp()
    {

    }

    private void liftOp()
    {

    }

}

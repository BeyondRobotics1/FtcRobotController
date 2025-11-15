package org.firstinspires.ftc.teamcode.decode.OpMode;

import android.graphics.Color;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
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
        STOP
    }

    private Timer actionTimer;
    private Timer gameTimer;

    private Shooter shooter;
    private Intake intake;
    private DriveTrain driveTrain;
    private Trigger trigger;

    private GamepadEx gamepad1Ex;
    private GamepadEx gamepad2Ex;

    private ToggleButtonReader leftBumper1;
    private boolean leftBumperToggled;

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
        trigger.open();

        actionTimer = new Timer();
        gameTimer = new Timer();

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        leftBumper1 = new ToggleButtonReader(
                gamepad1Ex, GamepadKeys.Button.LEFT_BUMPER
        );

        shootAutoCompleteMode = ShootAutoCompleteMode.STOP;

        telemetry.addLine("hardware initialization completed");


        telemetry.addLine("initializing LynxModule");
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry.addLine("LynxModule initialized");

        waitForStart();

        if(isStopRequested()) return;

        shooter.shoot();

        while(!isStopRequested() && opModeIsActive())
        {
            hubs.forEach(LynxModule::clearBulkCache);

            gamepad1Ex.readButtons();
            gamepad2Ex.readButtons();
            leftBumper1.readValue();

            intakeOp();
            shootOp();


            driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }

    private void intakeOp()
    {
        //
        if(leftBumper1.getState())
            leftBumperToggled = true;
        else
            leftBumperToggled = false;

        //intake control
        if (gamepad1Ex.isDown(GamepadKeys.Button.RIGHT_BUMPER)) //right bumper take in
            intake.SetIntakeMode(Intake.IntakeMode.OUT);
        else if (gamepad1Ex.wasJustPressed(GamepadKeys.Button.A))
            shootAutoCompleteMode = ShootAutoCompleteMode.START;
        else if (gamepad1Ex.wasJustReleased(GamepadKeys.Button.A))
            shootAutoCompleteMode = ShootAutoCompleteMode.STOP;
        else {
            if (leftBumperToggled) { //intake mode
                if(trigger.getHighColor() == Color.WHITE)
                    intake.SetIntakeMode(Intake.IntakeMode.IN);
                else
                    intake.SetIntakeMode(Intake.IntakeMode.HIN);
            }
            else
                intake.SetIntakeMode(Intake.IntakeMode.IDLE);
        }

        switch(shootAutoCompleteMode)
        {
            case START:
                actionTimer.resetTimer();
                shootAutoCompleteMode = ShootAutoCompleteMode.OPEN_TRIGGER;
                trigger.open();
                break;
            case OPEN_TRIGGER:
                if (actionTimer.getElapsedTime() > 250) {
                    actionTimer.resetTimer();
                    shootAutoCompleteMode = ShootAutoCompleteMode.TRIGGER_OPENED;
                }
            case TRIGGER_OPENED:
                intake.SetIntakeMode(Intake.IntakeMode.FEED);
                break;
            case STOP:
                intake.SetIntakeMode(Intake.IntakeMode.IDLE);
                //trigger.close();
                break;
        }
    }

    private void shootOp()
    {
        if(!gamepad1Ex.isDown(GamepadKeys.Button.X))
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

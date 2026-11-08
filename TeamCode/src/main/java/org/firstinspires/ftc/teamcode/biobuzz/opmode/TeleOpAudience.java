package org.firstinspires.ftc.teamcode.biobuzz.opmode;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.biobuzz.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.ShooterGroup;

import java.util.List;

@TeleOp(name = "BioBuzz TeleOp Audience", group = "A")

public class TeleOpAudience extends LinearOpMode {

    private DriveTrain driveTrain;
    private Intake intake;
    private ShooterGroup shooters;


    Gamepad.RumbleEffect softRumbleEffect;    // Use to build a custom rumble sequence
    Gamepad.RumbleEffect strongRumbleEffect;

    //status
    private Timer actionTimer;
    private Timer gameTimer;

    //update in real-time
    private boolean isBlueTeleOp = true;
    private boolean isIntakeOn;
    private boolean isShooterOn;
    private boolean enableAutoAiming = true;
    private boolean enableAutoShootingSpeed = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing drive train");
        telemetry.update();
        driveTrain = new DriveTrain(hardwareMap, this, false);

        telemetry.addLine("Initializing shooters");
        telemetry.update();
        shooters = new ShooterGroup(hardwareMap, this);

        telemetry.addLine("Initializing intake");
        telemetry.update();
        intake = new Intake(hardwareMap, this);



        actionTimer = new Timer();
        gameTimer = new Timer();

        telemetry.addLine("initializing LynxModule");
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry.addLine("LynxModule initialized");

        softRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.5, 100)  //
                .addStep(0.9, 0.9, 300)
                .addStep(0.5, 0.5, 100)
                .build();
        strongRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.5, 100)  //
                .addStep(1, 1, 350)
                .addStep(0.5, 0.5, 100)
                .addStep(1, 1, 350)
                .addStep(0.5, 0.5, 100)
                .build();

        isIntakeOn = false;
        isShooterOn = true;

        //select BLUE or RED
        while (!isStarted() && !isStopRequested()) {

            if(isBlueTeleOp)
                telemetry.addLine("TeleOp AUDIENCE Selected: BLUE BLUE BLUE");
            else
                telemetry.addLine("TeleOP AUDIENCE Selected: RED RED RED");

            telemetry.addLine("");
            telemetry.addLine("WARNING WARNING: Select the right TeleOp!!!");
            telemetry.addLine("Gamepad1.A: TeleOp AUDIENCE RED");
            telemetry.addLine("Gamepad1.B: TeleOp AUDIENCE BLUE");
            telemetry.addLine("-----------------------");

            if(gamepad1.a) {
                isBlueTeleOp = false;
            }
            else if (gamepad1.b) {
                isBlueTeleOp = true;
            }

            telemetry.update();
        }


        gameTimer.resetTimer();

        if(isStopRequested()) return;


        boolean isEndGame = false;
        int rumbleEndgame = 0;

        isIntakeOn = true;

        while(!isStopRequested() && opModeIsActive())
        {
            hubs.forEach(LynxModule::clearBulkCache);

            if(isBlueTeleOp)
                telemetry.addLine("TeleOp AUDIENCE Selected: BLUE BLUE BLUE");
            else
                telemetry.addLine("TeleOP AUDIENCE Selected: RED RED RED");

            telemetry.addLine("");

            driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


            if (gameTimer.getElapsedTimeSeconds() >= 80 && rumbleEndgame == 0)  {
                rumbleEndgame = 1;
                gamepad1.runRumbleEffect(strongRumbleEffect);
                gamepad2.runRumbleEffect(strongRumbleEffect);
            }


            if (gameTimer.getElapsedTimeSeconds() >= 100 && rumbleEndgame == 1)  {
                rumbleEndgame = 2;
                gamepad1.runRumbleEffect(strongRumbleEffect);
                gamepad2.runRumbleEffect(strongRumbleEffect);

                isEndGame = true;
            }

            telemetry.update();
        }
    }
}

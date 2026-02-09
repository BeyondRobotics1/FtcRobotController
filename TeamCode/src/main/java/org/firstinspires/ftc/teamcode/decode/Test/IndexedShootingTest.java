package org.firstinspires.ftc.teamcode.decode.Test;

import android.graphics.Color;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.OpMode.DecodeBlackBoard;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Trigger;

import java.util.List;

@TeleOp(name = "Indexed Shooting Test", group = "Decode Test")

public class IndexedShootingTest extends LinearOpMode {

    private Shooter shooter;
    private Indexer indexer;
    private Intake intake;
    private Trigger trigger;

    private Timer actionTimer;

    private boolean isIntakeOn;
    private boolean isShooterOn;
    private int shootingLocation; //1 slow, 2 middle, 3 fast

    int[] artifactColors; //0 - top, 1 - middle, 2 - bottom

    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Initializing indexer");
        telemetry.update();
        indexer = new Indexer(hardwareMap, this);

        telemetry.addLine("Initializing intake");
        intake = new Intake(hardwareMap, this);

        telemetry.addLine("Initializing trigger");
        trigger = new Trigger(hardwareMap);
        trigger.close();

        telemetry.addLine("Initializing shooter");
        shooter = new Shooter(hardwareMap, this, DecodeBlackBoard.RED);
        shooter.setShootingLocation(Shooter.ShootingLocation.MEDIUM);

        telemetry.addLine("hardware initialization completed");


        telemetry.addLine("initializing LynxModule");
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry.addLine("LynxModule initialized");

        isIntakeOn = false;
        isShooterOn = true;
        artifactColors = new int[3];
        artifactColors[0] = artifactColors[1] = artifactColors[2] = Color.WHITE;

        actionTimer = new Timer();

        while (!isStarted() && !isStopRequested()) {

            //read the obelisk

        }

        if (isStopRequested()) return;

        //let the flywheel spin for 500ms so
        //the PID controller won't draw too much batteries
        shooter.setPower(0.4);
        sleep(1000);

        boolean isInitialPinpointPositionSet = false;

        while (!isStopRequested() && opModeIsActive()) {
            if (!isInitialPinpointPositionSet) {
                //turret.setIMUPoseToRobotStartPose();
                isInitialPinpointPositionSet = true;
            }

            hubs.forEach(LynxModule::clearBulkCache);

            if (isShooterOn)
                shooter.shoot();
            else
                shooter.stop();

            if(gamepad1.aWasPressed())
            {
                IndexNoBallsAndShoot();
            }
            else if(gamepad1.bWasPressed())
            {
                indexOneBallAndShoot();
            }
            else if(gamepad1.yWasPressed())
            {
                indexTwoBallsAndShoot();
            }
        }
    }


    private void indexTwoBallsAndShoot() {
        intake.setIntakeMode(Intake.IntakeMode.IDLE);
        trigger.close();

        //index the first ball
        indexer.index(1);
        sleep(200);

        //move the second ball up
        intake.intake(0.9);
        sleep(300);
        intake.setIntakeMode(Intake.IntakeMode.IDLE);

        //index the second ball
        indexer.index(2);
        sleep(200);


        intake.intake(0.5);

        //shoot the third ball
        trigger.open();
        sleep(50);
        intake.setIntakeMode(Intake.IntakeMode.FEED);
        sleep(300);

        //deque the second ball;
        intake.intake(0.5);
        indexer.index(1);
        sleep(200);

        //shoot the second ball
        intake.setIntakeMode(Intake.IntakeMode.FEED);
        sleep(300);

        //deque the third ball
        intake.intake(0.5);
        indexer.index(0);
        sleep(200);

        //shoot the third ball
        intake.setIntakeMode(Intake.IntakeMode.FEED);
        sleep(300);
        trigger.close();
    }

    private void indexOneBallAndShoot() {
        intake.setIntakeMode(Intake.IntakeMode.IDLE);
        trigger.close();

        //index the first ball
        indexer.index(1);
        sleep(200);

        //move the second ball up
        intake.intake(0.9);
        sleep(300);
        intake.setIntakeMode(Intake.IntakeMode.IDLE);


        intake.intake(0.5);

        //shoot the third ball
        trigger.open();
        sleep(50);
        intake.setIntakeMode(Intake.IntakeMode.FEED);
        sleep(300);

        //deque the second ball;
        intake.intake(0.5);
        indexer.index(1);
        sleep(200);

        //shoot the second ball
        intake.setIntakeMode(Intake.IntakeMode.FEED);
        sleep(300);

        //deque the third ball
        intake.intake(0.5);
        indexer.index(0);
        sleep(200);

        //shoot the third ball
        intake.setIntakeMode(Intake.IntakeMode.FEED);
        sleep(300);
        trigger.close();
    }

    private void IndexNoBallsAndShoot()
    {
        intake.setIntakeMode(Intake.IntakeMode.IDLE);
        trigger.close();

        //move the second ball up
        intake.intake(0.9);
        sleep(300);
        intake.setIntakeMode(Intake.IntakeMode.IDLE);

        intake.intake(0.5);

        //shoot the third ball
        trigger.open();
        sleep(50);
        intake.setIntakeMode(Intake.IntakeMode.FEED);
        sleep(300);

        //deque the second ball;
        intake.intake(0.5);
        indexer.index(1);
        sleep(200);

        //shoot the second ball
        intake.setIntakeMode(Intake.IntakeMode.FEED);
        sleep(300);

        //deque the third ball
        intake.intake(0.5);
        indexer.index(0);
        sleep(200);

        //shoot the third ball
        intake.setIntakeMode(Intake.IntakeMode.FEED);
        sleep(300);
        trigger.close();
    }

}

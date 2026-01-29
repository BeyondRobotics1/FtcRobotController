package org.firstinspires.ftc.teamcode.decode.Test;

import android.graphics.Color;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.decode.OpMode.DecodeBlackBoard;
import org.firstinspires.ftc.teamcode.decode.OpMode.DecodeTeleOp;
import org.firstinspires.ftc.teamcode.decode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.decode.Subsystems.IMULocalizer;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Trigger;

import java.util.List;

@TeleOp(name = "Shooter Test", group = "Decode Test")

public class ShooterTest extends LinearOpMode {

    enum ShootAutoCompleteMode
    {
        START,
        //SHOOT, //shoot the balls
        //STOP,
        COMPLETED,
        //NOP //no op
    }

        //hardware
        private Shooter shooter;
        private Intake intake;
        private DriveTrain driveTrain;
        private Trigger trigger;
        private Turret turret;
        private Indexer indexer;

        Gamepad.RumbleEffect nearRumbleEffect;    // Use to build a custom rumble sequence
        Gamepad.RumbleEffect mediumRumbleEffect;
        Gamepad.RumbleEffect farRumbleEffect;

        //status
        private Timer actionTimer;
        private Timer gameTimer;

        private boolean isBlueTeleOp = true;
        private boolean isIntakeOn;
        private boolean isShooterOn;
        private int shootingLocation; //1 slow, 2 middle, 3 fast
        private boolean enableAutoAiming = true;

        int[] artifactColors; //0 - top, 1 - middle, 2 - bottom
        boolean isHeadingToGoal = false;
        IMULocalizer.RobotZone robotZone = IMULocalizer.RobotZone.NOT_IN_SHOOTING_ZONE;

        ShootAutoCompleteMode shootAutoCompleteMode;

        @Override
        public void runOpMode() throws InterruptedException {

            telemetry.addLine("Initializing indexer");
            telemetry.update();
            indexer = new Indexer(hardwareMap, this);

            telemetry.addLine("Initializing intake");
            intake = new Intake(hardwareMap, this);

            telemetry.addLine("Initializing trigger");
            trigger = new Trigger(hardwareMap);
            trigger.close();

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

            shootAutoCompleteMode = ShootAutoCompleteMode.COMPLETED;
            actionTimer = new Timer();
            gameTimer = new Timer();

            //waitForStart();
            while (!isStarted() && !isStopRequested()) {


                if (isBlueTeleOp)
                    telemetry.addLine("TeleOp Selected: BLUE");
                else
                    telemetry.addLine("TeleOP Selected: RED");

                telemetry.addLine("");
                telemetry.addLine("WARNING: Select the right TelelOp!!!");
                telemetry.addLine("Gamepad1.A: TeleOp RED");
                telemetry.addLine("Gamepad1.B: TeleOp BLUE");
                telemetry.addLine("-----------------------");

                if (gamepad1.a) {
                    isBlueTeleOp = false;
                } else if (gamepad1.b) {
                    isBlueTeleOp = true;
                }

                telemetry.update();
            }


            int alliance;
            if (isBlueTeleOp) {
                alliance = DecodeBlackBoard.BLUE;
                turret = new Turret(hardwareMap, this,
                        DecodeBlackBoard.BLUE_RESET_POSE,
                        DecodeBlackBoard.BLUE_TARGET_POSE,
                        alliance,
                        true,
                        true, false);

                telemetry.addLine("Initializing shooter");
                shooter = new Shooter(hardwareMap, this, alliance);

            } else {
                alliance = DecodeBlackBoard.RED;
                turret = new Turret(hardwareMap, this,
                        DecodeBlackBoard.RED_RESET_POSE,
                        DecodeBlackBoard.RED_TARGET_POSE,
                        alliance,
                        true,
                        true, false);
                telemetry.addLine("Initializing shooter");
                shooter = new Shooter(hardwareMap, this, alliance);
            }


            telemetry.addData("Turret initialized, camera is running:",
                    turret.isLimeLight3ARunning());

            telemetry.update();

            if (isStopRequested()) return;

            //let the flywheel spin for 500ms so
            //the PID controller won't draw too much batteries
            shooter.setPower(0.4);
            sleep(1200);

            Boolean isInitialPinpointPositionSet = false;

            while (!isStopRequested() && opModeIsActive()) {
                if (!isInitialPinpointPositionSet) {
                    turret.setIMUPoseToRobotStartPose();
                    isInitialPinpointPositionSet = true;
                }

                hubs.forEach(LynxModule::clearBulkCache);

                if (isBlueTeleOp)
                    telemetry.addLine("TeleOp Selected: BLUE");
                else
                    telemetry.addLine("TeleOP Selected: RED");

                telemetry.addLine("");

                //operate the intake
                intakeOp();

                //
                //turretOp();

                //operate the shooter
                shootOp();


                //operate the drive train
                //driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

                telemetry.update();
            }
        }

        private void intakeOp() {
            artifactColors = intake.detectArtifactColors();

            //RIGHT BUMPER button to toggle intake on/off
            if(gamepad1.rightBumperWasPressed())
                isIntakeOn = !isIntakeOn;

            //RIGHT TRIGGER to spit out extra ball
            if (Math.abs(gamepad1.right_trigger) > 0.5)
                intake.setIntakeMode(Intake.IntakeMode.OUT);
            else {
                if (isIntakeOn) { //intake mode

                    //shooting is done, intake now
                    if(shootAutoCompleteMode == ShootAutoCompleteMode.COMPLETED) {
                        //To save battery
                        //
                        //If there are three balls already, stop the intake
                        //
                        //if there're two balls already
                        //stop the 2nd intake motor and keep the front
                        //intake motor running only
                        //
                        if (artifactColors[0] != Color.WHITE &&
                                artifactColors[1] != Color.WHITE &&
                                artifactColors[2] != Color.WHITE)
                            intake.setIntakeMode(Intake.IntakeMode.IDLE);
                        else if (artifactColors[0] != Color.WHITE &&
                                artifactColors[1] != Color.WHITE)
                            intake.setIntakeMode(Intake.IntakeMode.HIN);
                        else
                            intake.intake(0.9);
                    }
                    else
                        intake.setIntakeMode(Intake.IntakeMode.FEED); //shoot at full speed
                }
                else
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
            }

            //LEFT BUMPER to start the shooting
            if (gamepad1.leftBumperWasPressed()) {
                shootAutoCompleteMode = ShootAutoCompleteMode.START;
                trigger.open();
            }
            else if (gamepad1.leftBumperWasReleased()) {
                shootAutoCompleteMode = ShootAutoCompleteMode.COMPLETED;
                trigger.close();
            }
        }

        private void shootOp() {
            //use gamepad1 X button to toggle
            //shooter motors
            if (gamepad1.leftBumperWasPressed())
                isShooterOn = !isShooterOn;

            //gamepad1 a, shoot from close position
            //gamepad1 b, shoot from medium position
            //gamepad1 y, shoot from far position
            if (gamepad1.aWasPressed()) {
                shooter.setShootingLocation(Shooter.ShootingLocation.NEAR);
                //gamepad1.runRumbleEffect(nearRumbleEffect);
                //gamepad2.runRumbleEffect(nearRumbleEffect);
            }
            else if (gamepad1.bWasPressed()) {
                shooter.setShootingLocation(Shooter.ShootingLocation.MEDIUM);
                //gamepad1.runRumbleEffect(farRumbleEffect);
                //gamepad2.runRumbleEffect(farRumbleEffect);
            }else if (gamepad1.yWasPressed()) {
                shooter.setShootingLocation(Shooter.ShootingLocation.FAR);
                //gamepad1.runRumbleEffect(farRumbleEffect);
                //gamepad2.runRumbleEffect(farRumbleEffect);
            } else if (gamepad1.xWasPressed()) {
                shooter.setShootingLocation(Shooter.ShootingLocation.OUT_ZONE);
                //gamepad1.runRumbleEffect(mediumRumbleEffect);
                //gamepad2.runRumbleEffect(mediumRumbleEffect);
            }


            if (isShooterOn)
                shooter.shoot();
            else
                shooter.stop();
        }

        private void turretOp() {
            isHeadingToGoal = turret.isHeadingToGoal();
            robotZone = turret.getRobotZone();

            //Keep gamepad2 left_bumper button down to give a new known position to the pinpoint
            if(gamepad2.left_bumper)
                turret.resetIMUPose();

            //use gamepad2 x button to disable or enable auto aiming
            if(gamepad2.xWasPressed())
                enableAutoAiming = !enableAutoAiming;

            if(enableAutoAiming)
                turret.autoAim(true);
            else {
                turret.autoAim(false);

                turret.resetTurretHeading();
            }
        }

        private void liftOp() {

        }
    }

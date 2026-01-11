package org.firstinspires.ftc.teamcode.decode.OpMode;

import android.graphics.Color;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.decode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.decode.Subsystems.IMULocalizer;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Trigger;

import java.util.List;

@TeleOp(name = "Decode TeleOp", group = "A")

public class DecodeTeleOp extends LinearOpMode {

    public enum ShootAutoCompleteMode
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
    private Lift lift;

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

    int[] artifactColors; //0 - top, 1 - middle, 2 - bottom
    boolean isHeadingToGoal = false;
    IMULocalizer.RobotZone robotZone = IMULocalizer.RobotZone.NOT_IN_SHOOTING_ZONE;

    Pose2D robotPose;

    ShootAutoCompleteMode shootAutoCompleteMode;


    //field centric driving by default
    //use dpad up to toggle on/off
    boolean fieldCentric = true;
    int rumbleReady = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing drive train");
        telemetry.update();
        driveTrain = new DriveTrain(hardwareMap, this, false);

        telemetry.addLine("Initializing indexer");
        telemetry.update();
        indexer = new Indexer(hardwareMap, this);

        telemetry.addLine("Initializing intake");
        intake = new Intake(hardwareMap, this);

        telemetry.addLine("Initializing trigger");
        trigger = new Trigger(hardwareMap);
        trigger.close();

        telemetry.addLine("Initializing lift");
        lift = new Lift(hardwareMap);

        actionTimer = new Timer();
        gameTimer = new Timer();

        shootAutoCompleteMode = ShootAutoCompleteMode.COMPLETED;

        telemetry.addLine("hardware initialization completed");


        telemetry.addLine("initializing LynxModule");
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry.addLine("LynxModule initialized");

        softRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.1, 0.1, 100)  //
                .build();
        strongRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.5, 100)  //
                .addStep(1, 1, 200)
                .addStep(0.5, 0.5, 100)
                .build();

        isIntakeOn = false;
        isShooterOn = true;
        artifactColors = new int[3];
        artifactColors[0] = artifactColors[1] = artifactColors[2] = Color.WHITE;

        //waitForStart();
        while (!isStarted() && !isStopRequested()) {

            if(robotPose == null)
                robotPose = DecodeBlackBoard.robotAutoEndPose();

            if(isBlueTeleOp)
                telemetry.addLine("TeleOp Selected: BLUE BLUE BLUE");
            else
                telemetry.addLine("TeleOP Selected: RED RED RED");

            telemetry.addLine("");
            telemetry.addLine("WARNING WARNING: Select the right TelelOp!!!");
            telemetry.addLine("Gamepad1.A: TeleOp RED");
            telemetry.addLine("Gamepad1.B: TeleOp BLUE");
            telemetry.addLine("-----------------------");
            telemetry.addData("Auto end X (Inch):", robotPose.getX(DistanceUnit.INCH));
            telemetry.addData("Auto end Y (Inch):", robotPose.getY(DistanceUnit.INCH));
            telemetry.addData("Auto end Heading (Degree) :", robotPose.getHeading(AngleUnit.DEGREES));

            if(gamepad1.a) {
                isBlueTeleOp = false;
            }
            else if (gamepad1.b) {
                isBlueTeleOp = true;
            }

            telemetry.update();
        }

        gameTimer.resetTimer();
        int rumbleEndgame = 0;


        int alliance;
        if(isBlueTeleOp) {
            alliance = DecodeBlackBoard.BLUE;
            turret = new Turret(hardwareMap, this,
                    robotPose,
                    DecodeBlackBoard.BLUE_TARGET_POSE,
                    alliance,
                    true,
                    true, false);

            telemetry.addLine("Initializing shooter");
            shooter = new Shooter(hardwareMap, this, alliance);
        }
        else {
            alliance = DecodeBlackBoard.RED;
            turret = new Turret(hardwareMap, this,
                    robotPose,
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

        if(isStopRequested()) return;

        //let the flywheel spin for 500ms so
        //the PID controller won't draw too much batteries
        shooter.setPower(0.4);
        sleep(1000);

        boolean isInitialPinpointPositionSet = false;

        while(!isStopRequested() && opModeIsActive())
        {
            if(!isInitialPinpointPositionSet)
            {
                turret.setIMUPoseToRobotStartPose();
                isInitialPinpointPositionSet = true;
            }

            hubs.forEach(LynxModule::clearBulkCache);

            if(isBlueTeleOp)
                telemetry.addLine("TeleOp Selected: BLUE BLUE BLUE");
            else
                telemetry.addLine("TeleOP Selected: RED RED RED");

            telemetry.addLine("");

            //operate the intake
            intakeOp();

            //
            turretOp();

            //operate the shooter
            shootOp();


            ////DPAD UP to toggle field centric or robot centric driving
            //if(gamepad1.dpadUpWasPressed())
            //    fieldCentric = !fieldCentric;

            //if(fieldCentric)
            //    driveTrain.setPower2(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x,
            //            Math.toRadians(180+turret.getBotHeadingDegrees()));
            //else
                driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gameTimer.getElapsedTimeSeconds() >= 80 && rumbleEndgame == 0)  {
                rumbleEndgame = 1;
                gamepad1.runRumbleEffect(strongRumbleEffect);
                gamepad2.runRumbleEffect(strongRumbleEffect);
            }

            telemetry.update();
        }
    }

    private void intakeOp()
    {
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
                            artifactColors[2] != Color.WHITE) {
                        intake.setIntakeMode(Intake.IntakeMode.IDLE);

                        if((rumbleReady % 10) == 0) {
                            gamepad1.runRumbleEffect(softRumbleEffect);
                            gamepad2.runRumbleEffect(softRumbleEffect);
                        }

                        rumbleReady++;

                        if(rumbleReady >= 100000)
                            rumbleReady = 0;
                    }
                    else if (artifactColors[0] != Color.WHITE &&
                            artifactColors[1] != Color.WHITE)
                        intake.setIntakeMode(Intake.IntakeMode.HIN);
                    else
                        intake.intake(0.925);
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

//        ////auto complete shooting
//        if(shootAutoCompleteMode == ShootAutoCompleteMode.START ||
//            shootAutoCompleteMode == ShootAutoCompleteMode.COMPLETED)
//        {
//            switch (shootAutoCompleteMode) {
//                case START:
//                    actionTimer.resetTimer();
//                    shootAutoCompleteMode = ShootAutoCompleteMode.SHOOT;
//                    trigger.open();
//                    break;
////                case SHOOT:
////                    if (actionTimer.getElapsedTime() > 250) {
////                        actionTimer.resetTimer();
////                        intake.setIntakeMode(Intake.IntakeMode.FEED);
////                        shootAutoCompleteMode = ShootAutoCompleteMode.STOP;
////                    }
////                    break;
////                case STOP:
////                    if (actionTimer.getElapsedTime() > 2000) {
////                        actionTimer.resetTimer();
////                        shootAutoCompleteMode = ShootAutoCompleteMode.COMPLETED;
////                    }
////                    break;
//                case COMPLETED:
//                    trigger.close();
//                    break;
//            }
//        }
    }

    private void turretOp()
    {
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

    private void shootOp()
    {
        //press DPAD-UP to enable auto shooting speed
        if(gamepad1.dpadUpWasPressed())
            enableAutoShootingSpeed = true;

        //press DPA-DOWN to disable auto shooting speed
        if(gamepad1.dpadDownWasPressed())
            enableAutoShootingSpeed = false;

        //auto shooting speed is enabled
        //Only adjust shooting speed if there're balls in our robot
        if(enableAutoShootingSpeed) {
            if (artifactColors[0] != Color.WHITE ||
                    artifactColors[1] != Color.WHITE ||
                    artifactColors[2] != Color.WHITE) {
                if (isHeadingToGoal)
                {
                    if (robotZone == IMULocalizer.RobotZone.OUT_SHOOTING_ZONE)
                        shooter.setShootingLocation(Shooter.ShootingLocation.OUT_ZONE);
                    else if (robotZone == IMULocalizer.RobotZone.NEAR_SHOOTING_ZONE)
                        shooter.setShootingLocation(Shooter.ShootingLocation.NEAR);
                    else if (robotZone == IMULocalizer.RobotZone.MEDIUM_SHOOTING_ZONE)
                        shooter.setShootingLocation(Shooter.ShootingLocation.MEDIUM);
                    else if (robotZone == IMULocalizer.RobotZone.FAR_SHOOTING_ZONE)
                        shooter.setShootingLocation(Shooter.ShootingLocation.FAR);
                }
            }
        }
        else {
            //gamepad1 a, shoot from near position
            //gamepad1 b, shoot from medium position
            //gamepad1 y, shoot from far position
            //gamepad1 x, shoot from OUT_ZONE position
            if (gamepad1.aWasPressed()) {
                shooter.setShootingLocation(Shooter.ShootingLocation.NEAR);

            } else if (gamepad1.xWasPressed()) {
                shooter.setShootingLocation(Shooter.ShootingLocation.OUT_ZONE);
            } else if (gamepad1.yWasPressed()) {
                shooter.setShootingLocation(Shooter.ShootingLocation.FAR);
            } else if (gamepad1.bWasPressed()) {
                shooter.setShootingLocation(Shooter.ShootingLocation.MEDIUM);
            }
        }

        if(isShooterOn)
            shooter.shoot();
        else
            shooter.stop();
    }

    private void liftOp()
    {

    }
}

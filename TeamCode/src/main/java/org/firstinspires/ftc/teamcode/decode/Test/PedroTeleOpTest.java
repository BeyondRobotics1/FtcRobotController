package org.firstinspires.ftc.teamcode.decode.Test;

import android.graphics.Color;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.decode.OpMode.DecodeBlackBoard;
import org.firstinspires.ftc.teamcode.decode.OpMode.DecodeTeleOpNear;
import org.firstinspires.ftc.teamcode.decode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.decode.Subsystems.IMULocalizer;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Trigger;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.function.Supplier;

@TeleOp(name = "Decode Field Centric TeleOp Test", group = "Decode Test")
public class PedroTeleOpTest extends LinearOpMode {


    public enum ShootAutoCompleteMode
    {
        START,
        //SHOOT, //shoot the balls
        //STOP,
        COMPLETED,
        //NOP //no op
    }

    public enum LiftMode
    {
        NONE,
        START,
        PTO_ENGAGED,
        COMPLETED
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

    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    public static Pose openGatePose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;

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

    DecodeTeleOpNear.ShootAutoCompleteMode shootAutoCompleteMode;
    DecodeTeleOpNear.LiftMode liftMode;

    //field centric driving by default
    //use dpad up to toggle on/off
    boolean fieldCentric = true;
    int rumbleReady = 0;

    boolean is_near = true;

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

        shootAutoCompleteMode = DecodeTeleOpNear.ShootAutoCompleteMode.COMPLETED;
        liftMode = DecodeTeleOpNear.LiftMode.NONE;

        telemetry.addLine("hardware initialization completed");


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
        artifactColors = new int[3];
        artifactColors[0] = artifactColors[1] = artifactColors[2] = Color.WHITE;


        //waitForStart();
        while (!isStarted() && !isStopRequested()) {

            if(gamepad1.a) {
                isBlueTeleOp = false;
            }
            else if (gamepad1.b) {
                isBlueTeleOp = true;
            }

            if(gamepad1.x)
                is_near = true;
            if(gamepad1.y)
                is_near = false;


            if(isBlueTeleOp) {

                if(is_near)
                    telemetry.addLine("TeleOp NEAR Selected: BLUE BLUE BLUE");
                else
                    telemetry.addLine("TeleOp FAR Selected: BLUE BLUE BLUE");

                if(robotPose == null) {
                    if(is_near)
                        robotPose = DecodeBlackBoard.BLUE_NEAR_RESET_POSE;
                    else
                        robotPose = DecodeBlackBoard.BLUE_FAR_RESET_POSE;
                }
            }
            else {
                if(is_near)
                    telemetry.addLine("TeleOp NEAR Selected: RED RED RED");
                else
                    telemetry.addLine("TeleOp FAR Selected: RED RED RED");

                if (robotPose == null) {
                    if(is_near)
                        robotPose = DecodeBlackBoard.RED_NEAR_RESET_POSE;
                    else
                        robotPose = DecodeBlackBoard.RED_FAR_RESET_POSE;
                }
            }

            telemetry.addLine("");
            telemetry.addLine("WARNING: Select the right TelelOp!!!");
            telemetry.addLine("Gamepad1.A: TeleOp RED");
            telemetry.addLine("Gamepad1.B: TeleOp BLUE");
            telemetry.addLine("Gamepad1.X: TeleOp NEAR");
            telemetry.addLine("Gamepad1.Y: TeleOp FAR");
            telemetry.addLine("-----------------------");
            telemetry.addData("Auto end X (Inch):", robotPose.getX(DistanceUnit.INCH));
            telemetry.addData("Auto end Y (Inch):", robotPose.getY(DistanceUnit.INCH));
            telemetry.addData("Auto end Heading (Degree) :", robotPose.getHeading(AngleUnit.DEGREES));


            telemetry.update();

        }

        gameTimer.resetTimer();
        int rumbleEndgame = 0;

        int alliance;
        if(isBlueTeleOp) {
            alliance = DecodeBlackBoard.BLUE;

            if(is_near) {
                turret = new Turret(hardwareMap, this,
                        DecodeBlackBoard.BLUE_NEAR_RESET_POSE,
                        DecodeBlackBoard.BLUE_TARGET_POSE,
                        alliance,
                        true,
                        true, false);

                startingPose = new Pose(DecodeBlackBoard.BLUE_NEAR_RESET_POSE.getX(DistanceUnit.INCH),
                        DecodeBlackBoard.BLUE_NEAR_RESET_POSE.getY(DistanceUnit.INCH),
                        Math.toRadians(DecodeBlackBoard.BLUE_NEAR_RESET_POSE.getHeading(AngleUnit.DEGREES))
                );

                openGatePose = new Pose(DecodeBlackBoard.BLUE_OPEN_GATE_POSE.getX(DistanceUnit.INCH),
                        DecodeBlackBoard.BLUE_OPEN_GATE_POSE.getY(DistanceUnit.INCH),
                        Math.toRadians(DecodeBlackBoard.BLUE_OPEN_GATE_POSE.getHeading(AngleUnit.DEGREES))
                );
            }
            else
            {
                turret = new Turret(hardwareMap, this,
                        DecodeBlackBoard.BLUE_FAR_RESET_POSE,
                        DecodeBlackBoard.BLUE_TARGET_POSE,
                        alliance,
                        true,
                        true, false);

                startingPose = new Pose(DecodeBlackBoard.BLUE_FAR_RESET_POSE.getX(DistanceUnit.INCH),
                        DecodeBlackBoard.BLUE_FAR_RESET_POSE.getY(DistanceUnit.INCH),
                        Math.toRadians(DecodeBlackBoard.BLUE_FAR_RESET_POSE.getHeading(AngleUnit.DEGREES))
                );

                openGatePose = new Pose(DecodeBlackBoard.BLUE_OPEN_GATE_POSE.getX(DistanceUnit.INCH),
                        DecodeBlackBoard.BLUE_OPEN_GATE_POSE.getY(DistanceUnit.INCH),
                        Math.toRadians(DecodeBlackBoard.BLUE_OPEN_GATE_POSE.getHeading(AngleUnit.DEGREES))
                );
            }

            telemetry.addLine("Initializing shooter");
            shooter = new Shooter(hardwareMap, this, alliance);
        }
        else {
            alliance = DecodeBlackBoard.RED;

            if(is_near) {
                turret = new Turret(hardwareMap, this,
                        DecodeBlackBoard.RED_NEAR_RESET_POSE,
                        DecodeBlackBoard.RED_TARGET_POSE,
                        alliance,
                        true,
                        true, false);

                startingPose = new Pose(DecodeBlackBoard.RED_NEAR_RESET_POSE.getX(DistanceUnit.INCH),
                        DecodeBlackBoard.RED_NEAR_RESET_POSE.getY(DistanceUnit.INCH),
                        Math.toRadians(DecodeBlackBoard.RED_NEAR_RESET_POSE.getHeading(AngleUnit.DEGREES))
                );

                openGatePose = new Pose(DecodeBlackBoard.RED_OPEN_GATE_POSE.getX(DistanceUnit.INCH),
                        DecodeBlackBoard.RED_OPEN_GATE_POSE.getY(DistanceUnit.INCH),
                        Math.toRadians(DecodeBlackBoard.RED_OPEN_GATE_POSE.getHeading(AngleUnit.DEGREES))
                );
            }
            else
            {
                turret = new Turret(hardwareMap, this,
                        DecodeBlackBoard.RED_FAR_RESET_POSE,
                        DecodeBlackBoard.RED_TARGET_POSE,
                        alliance,
                        true,
                        true, false);

                startingPose = new Pose(DecodeBlackBoard.RED_FAR_RESET_POSE.getX(DistanceUnit.INCH),
                        DecodeBlackBoard.RED_FAR_RESET_POSE.getY(DistanceUnit.INCH),
                        Math.toRadians(DecodeBlackBoard.RED_FAR_RESET_POSE.getHeading(AngleUnit.DEGREES))
                );

                openGatePose = new Pose(DecodeBlackBoard.RED_OPEN_GATE_POSE.getX(DistanceUnit.INCH),
                        DecodeBlackBoard.RED_OPEN_GATE_POSE.getY(DistanceUnit.INCH),
                        Math.toRadians(DecodeBlackBoard.RED_OPEN_GATE_POSE.getHeading(AngleUnit.DEGREES))
                );
            }

            telemetry.addLine("Initializing shooter");
            shooter = new Shooter(hardwareMap, this, alliance);
        }

        telemetry.addData("Turret initialized, camera is running:",
                turret.isLimeLight3ARunning());

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, openGatePose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, openGatePose.getHeading(), 0.8))
                .build();

        telemetry.update();

        if(isStopRequested()) return;

        follower.startTeleopDrive();

        //let the flywheel spin for 500ms so
        //the PID controller won't draw too much batteries
        shooter.setPower(0.4);
        sleep(1000);

        boolean isEndGame = false;

        boolean isInitialPinpointPositionSet = false;

        isIntakeOn = false;

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

            //once per loop
            follower.update();

            //operate the intake
            intakeOp();

            //
            turretOp();

            //operate the shooter
            shootOp();

            //operate the lift
            //if(isEndGame)
            liftOp();

            ////DPAD UP to toggle field centric or robot centric driving
            //if(gamepad1.dpadUpWasPressed())
            //    fieldCentric = !fieldCentric;

            //if(fieldCentric)
            //    driveTrain.setPower2(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x,
            //            Math.toRadians(180+turret.getBotHeadingDegrees()));
            //else
            if(liftMode == DecodeTeleOpNear.LiftMode.NONE && !gamepad1.dpad_down && !automatedDrive) {
                //driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

                follower.setTeleOpDrive(
                        gamepad1.left_stick_y, //-
                        gamepad1.left_stick_x, //-
                        -gamepad1.right_stick_x, //-
                        false // Robot Centric //true
                );

//                follower.setTeleOpDrive(
//                        -gamepad1.left_stick_y, //-
//                        -gamepad1.left_stick_x, //-
//                        -gamepad1.right_stick_x, //-
//                        true // Robot Centric //true
//                );
            }

            //Automated PathFollowing
            if (gamepad1.aWasPressed()) {
                follower.followPath(pathChain.get());
                automatedDrive = true;
            }
            //Stop automated following if the follower is done
            if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
                follower.startTeleopDrive();
                automatedDrive = false;
            }

            if (gameTimer.getElapsedTimeSeconds() >= 80 && rumbleEndgame == 0)  {
                rumbleEndgame = 1;
                gamepad1.runRumbleEffect(softRumbleEffect);
                gamepad2.runRumbleEffect(softRumbleEffect);
            }


            if (gameTimer.getElapsedTimeSeconds() >= 100 && rumbleEndgame == 1)  {
                rumbleEndgame = 2;
                gamepad1.runRumbleEffect(strongRumbleEffect);
                gamepad2.runRumbleEffect(strongRumbleEffect);

                isEndGame = true;
            }

            Pose p = follower.getPose();
            telemetry.addData("Follower X", p.getX());
            telemetry.addData("Follower Y", p.getY());
            telemetry.addData("Follower Heading",  Math.toDegrees(p.getHeading()));

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
                if(shootAutoCompleteMode == DecodeTeleOpNear.ShootAutoCompleteMode.COMPLETED) {
                    //To save battery
                    //
                    //If there are three balls already, stop the intake
                    //
                    //if there're two balls already
                    //stop the 2nd intake motor and keep the front
                    //intake motor running only
                    //
                    if (intake.detectedArtifacts() == 3) {
                        intake.setIntakeMode(Intake.IntakeMode.IDLE);
                        //intake.intake(0.95, 0);//0
                        intake.setLedColor(Intake.LED_GREEN);
                    }
                    else if (intake.detectedArtifacts() == 2) {
                        intake.setLedColor(Intake.LED_YELLOW);
                        intake.setIntakeMode(Intake.IntakeMode.HIN);
                    }
                    else if (intake.detectedArtifacts() == 1){
                        intake.setLedColor(Intake.LED_ORANGE);
                        intake.intake(0.95, 0.925);//0.925
                    }
                    else
                    {
                        intake.setLedColor(Intake.LED_OFF);
                        intake.intake(0.95,0.925);//0.925
                    }
                }
                else
                    intake.setIntakeMode(Intake.IntakeMode.FEED); //shoot at full speed
            }
            else
                intake.setIntakeMode(Intake.IntakeMode.IDLE);
        }

        //LEFT BUMPER to start the shooting
        if (gamepad1.leftBumperWasPressed()) {
            shootAutoCompleteMode = DecodeTeleOpNear.ShootAutoCompleteMode.START;
            trigger.open();
        }
        else if (gamepad1.leftBumperWasReleased()) {
            shootAutoCompleteMode = DecodeTeleOpNear.ShootAutoCompleteMode.COMPLETED;
            trigger.close();
        }
    }

    private void turretOp()
    {
        isHeadingToGoal = turret.isHeadingToGoal();
        robotZone = turret.getRobotZone();

        //Keep gamepad2 left_bumper button down to give a new known position to the pinpoint
        if(gamepad2.left_bumper) {

            if(is_near) {
                if(isBlueTeleOp)
                    turret.setIMUPose(DecodeBlackBoard.BLUE_NEAR_RESET_POSE);
                else
                    turret.setIMUPose(DecodeBlackBoard.RED_NEAR_RESET_POSE);
            }
            else {
                if(isBlueTeleOp)
                    turret.setIMUPose(DecodeBlackBoard.BLUE_FAR_RESET_POSE);
                else
                    turret.setIMUPose(DecodeBlackBoard.RED_FAR_RESET_POSE);
            }
        }

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
        //GAMEPAD 2 press DPAD-UP to enable auto shooting speed
        if(gamepad2.dpadUpWasPressed())
            enableAutoShootingSpeed = true;

        //GAMEPAD 2 press DPAD-DOWN to disable auto shooting speed
        if(gamepad2.dpadDownWasPressed())
            enableAutoShootingSpeed = false;

        //auto shooting speed is enabled
        //Only adjust shooting speed if there're balls in our robot
        if(enableAutoShootingSpeed) {
            if (intake.detectedArtifacts() > 0) {
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
                    else if (robotZone == IMULocalizer.RobotZone.FAR_FAR_SHOOTING_ZONE)
                        shooter.setShootingLocation(Shooter.ShootingLocation.FAR_FAR);
                }
            }
        }
        else {
            if(is_near)
                shooter.setShootingLocation(Shooter.ShootingLocation.MEDIUM);
            else
                shooter.setShootingLocation(Shooter.ShootingLocation.OUT_ZONE);
        }

        //gamepad2 a, index 2
        //gamepad2 b, index 1
        //otherwise, index 0
        if (gamepad2.b) {
            indexer.index(2);
        } else if (gamepad2.a) {
            indexer.index(1);
        } else  {
            indexer.index(0);
        }

        if(isShooterOn)
            shooter.doFlyWheelVelocityPID();
        else
            shooter.stop();
    }

    private void liftOp()
    {
        //gamepad 1 dpad down for auto parking till
        //the red or blue line
        if(gamepad1.dpad_down)
            driveTrain.driveToLine();


        //gamepad 1 dpad up pressed
        if (gamepad1.dpadUpWasPressed()) {
            if(liftMode == DecodeTeleOpNear.LiftMode.NONE ||
                    liftMode == DecodeTeleOpNear.LiftMode.COMPLETED) {
                liftMode = DecodeTeleOpNear.LiftMode.START;
                actionTimer.resetTimer();
            }
        }//gamepad 1 dpad up released
        else if (gamepad1.dpadUpWasReleased())
            liftMode = DecodeTeleOpNear.LiftMode.COMPLETED;

        switch (liftMode)
        {
            case START:
                isIntakeOn = false;
                isShooterOn = false;
                enableAutoAiming = false;
                enableAutoShootingSpeed =false;

                lift.releaseHolder(true);
                lift.engageClutch(true);
                liftMode = DecodeTeleOpNear.LiftMode.PTO_ENGAGED;
                break;
            case PTO_ENGAGED:
                if(actionTimer.getElapsedTime() > 800)
                {
                    driveTrain.liftUp(1.0);
                }
                break;
            case COMPLETED:
                driveTrain.liftUp(0.0);
                lift.engageClutch(false);
                break;
        }


    }
}

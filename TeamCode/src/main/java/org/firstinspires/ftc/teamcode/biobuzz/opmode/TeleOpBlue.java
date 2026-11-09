package org.firstinspires.ftc.teamcode.biobuzz.opmode;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.IMULocalizer;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.ShooterGroup;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.TriggerGroup;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Turret;

import java.util.List;

@TeleOp(name = "BioBuzz TeleOp BLUE", group = "A")

public class TeleOpBlue extends LinearOpMode {

    private DriveTrain driveTrain;
    private Intake intake;
    private ShooterGroup shooters;
    private TriggerGroup triggers;
    private Turret turret;

    Gamepad.RumbleEffect softRumbleEffect;    // Use to build a custom rumble sequence
    Gamepad.RumbleEffect strongRumbleEffect;

    //status
    private Timer actionTimer;
    private Timer gameTimer;

    //update in real-time
    private boolean isIntakeOn;
    private boolean isShooterOn;
    private boolean enableAutoAiming = true;
    private boolean isHeadingToGoal = false;
    private boolean enableAutoShootingSpeed = true;

    private int rumbleEndgame = 0;

    Pose2D robotPose; //robot start pose
    IMULocalizer.RobotZone robotZone = IMULocalizer.RobotZone.NOT_IN_SHOOTING_ZONE;
    ShootAutoCompleteState shootAutoCompleteMode = ShootAutoCompleteState.COMPLETED;

    @Override
    public void runOpMode() throws InterruptedException {

        initSubsystems();

        telemetry.addLine("initializing LynxModule");
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry.addLine("LynxModule initialized");
        telemetry.update();

        isIntakeOn = false;
        isShooterOn = true;

        //select BLUE or RED
        boolean isAudienceSideTeleOp = true;
        while (!isStarted() && !isStopRequested()) {

            if(robotPose == null)
                robotPose = BioBuzzDashboard.robotAutoEndPose(blackboard);

            if(isAudienceSideTeleOp)
                telemetry.addLine("TeleOp BLUE AUDIENCE selected");
            else
                telemetry.addLine("TeleOP BLUE SCORE selected");

            telemetry.addLine("");
            telemetry.addLine("WARNING WARNING: Select the right TeleOp!!!");
            telemetry.addLine("Gamepad1.A: TeleOp BLUE AUDIENCE");
            telemetry.addLine("Gamepad1.B: TeleOp BLUE SCORE");
            telemetry.addLine("-----------------------");

            telemetry.addLine("");
            telemetry.addLine("WARNING WARNING: Select the right TeleOp!!!");
            telemetry.addLine("Gamepad1.A: TeleOp BLUE AUDIENCE");
            telemetry.addLine("Gamepad1.B: TeleOp BLUE SCORE");
            telemetry.addLine("-----------------------");
            telemetry.addData("Auto end X (Inch):", robotPose.getX(DistanceUnit.INCH));
            telemetry.addData("Auto end Y (Inch):", robotPose.getY(DistanceUnit.INCH));
            telemetry.addData("Auto end Heading (Degree) :", robotPose.getHeading(AngleUnit.DEGREES));

            if(gamepad1.a) {
                isAudienceSideTeleOp = true;
            }
            else if (gamepad1.b) {
                isAudienceSideTeleOp = false;
            }

            telemetry.update();
        }

        gameTimer.resetTimer();

        int alliance = BioBuzzDashboard.kAllianceRed;
        if(isAudienceSideTeleOp) {
            //no pose read
            if(robotPose == null || robotPose.getX(DistanceUnit.INCH) < 10.)
                robotPose = BioBuzzDashboard.kBlueAutoParkPoseAudience;

            turret = new Turret(hardwareMap, this, robotPose, alliance, false,false);

            //for LimeLight target angle
            //turret.setTargetAngleDegree(Turret.TARGET_ANGLE_DEGREE_BLUE_NEAR);
        }
        else
        {
            //no pose read
            if(robotPose == null || robotPose.getX(DistanceUnit.INCH) < 10.)
                robotPose = BioBuzzDashboard.kBlueAutoParkPoseScore;

            turret = new Turret(hardwareMap, this, robotPose, alliance, false,false);

            //for LimeLight target angle
            //turret.setTargetAngleDegree(Turret.TARGET_ANGLE_DEGREE_BLUE_NEAR);
        }


        if(isStopRequested()) return;


        //let the flywheel spin for 1000ms so
        //the PID controller won't draw too much battery
        shooters.setPower(0.9);
        sleep(150);//1000

        ///let the PID work for a while
        for (int i = 0; i < 30; i++) {
            shooters.doFlyWheelPID();
            sleep(10);
        }

        boolean isInitialPinpointPositionSet = false;

        isIntakeOn = true;

        while(!isStopRequested() && opModeIsActive())
        {
            if(!isInitialPinpointPositionSet)
            {
                //turret.setIMUPoseToRobotStartPose();
                isInitialPinpointPositionSet = true;
            }

            hubs.forEach(LynxModule::clearBulkCache);

            if(isAudienceSideTeleOp)
                telemetry.addLine("TeleOp BLUE AUDIENCE selected");
            else
                telemetry.addLine("TeleOp BLUE SCORE selected");

            telemetry.addLine("");

            driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            //operate the intake
            intakeOp();

            //
            turretOp();

            //operate the shooter
            shootOp();


            doRumble();

            telemetry.update();
        }
    }

    private void intakeOp()
    {
        //RIGHT BUMPER button to toggle intake on/off
        if(gamepad1.rightBumperWasPressed())
            isIntakeOn = !isIntakeOn;

        //RIGHT TRIGGER to spit out extra ball
        if (Math.abs(gamepad1.right_trigger) > 0.5)
            intake.setIntakeMode(Intake.IntakeMode.OUT);
        else {
            if (isIntakeOn) { //intake mode

                //shooting is done, intake now
                if(shootAutoCompleteMode == ShootAutoCompleteState.COMPLETED) {
                    intake.setIntakeMode(Intake.IntakeMode.IN);
//                    //To save battery
//                    //
//                    //If there are three balls already, stop the intake
//                    //
//                    //if there're two balls already
//                    //stop the 2nd intake motor and keep the front
//                    //intake motor running only
//                    //
//                    if (intake.detectedArtifacts() == 3) {
//                        intake.intake(0.1, 0);//0
//                        intake.setLedColor(Intake.LED_GREEN);
//                    }
//                    else if (intake.detectedArtifacts() == 2) {
//                        intake.setLedColor(Intake.LED_YELLOW);
//                        intake.setIntakeMode(Intake.IntakeMode.INTAKE_ONLY);
//                    }
//                    else if (intake.detectedArtifacts() == 1){
//                        intake.setLedColor(Intake.LED_ORANGE);
//                        intake.intake(0.95,0.925);//0.925
//                    }
//                    else
//                    {
//                        intake.setLedColor(Intake.LED_OFF);
//                        intake.intake(0.95,0.925);//0.925
//                    }
                }
                else
                    intake.setIntakeMode(Intake.IntakeMode.FEED); //shoot at full speed
            }
            else
                intake.setIntakeMode(Intake.IntakeMode.IDLE);
        }


        //LEFT BUMPER to start the shooting
        if (gamepad1.leftBumperWasPressed()) {
            shootAutoCompleteMode = ShootAutoCompleteState.START;
            triggers.open();
        }
        else if (gamepad1.leftBumperWasReleased()) {
            shootAutoCompleteMode = ShootAutoCompleteState.COMPLETED;
            triggers.close();
        }
    }


    private void turretOp()
    {
        isHeadingToGoal = turret.isHeadingToGoal();
        robotZone = turret.getRobotZone();

        //Keep gamepad2 left_bumper button down to give a new known position to the pinpoint
        if(gamepad2.left_bumper)
            turret.setIMUPose(BioBuzzDashboard.kRedIMUResetPoseAudience);

        //use gamepad2 x button to disable or enable auto aiming
        if(gamepad2.xWasPressed()) {
            enableAutoAiming = !enableAutoAiming;

            if (!enableAutoAiming)
                turret.resetTurretHeading();
        }

        //turret.autoAim(enableAutoAiming);
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
//                    if (robotZone == IMULocalizer.RobotZone.OUT_SHOOTING_ZONE) {
//                        if(isBlueTeleOp)
//                            shooters.setShootingLocation(Shooter.ShootingLocation.OUT_ZONE_BLUE);
//                        else
//                            shooters.setShootingLocation(Shooter.ShootingLocation.OUT_ZONE_RED);
//                    }
//                    else if (robotZone == IMULocalizer.RobotZone.NEAR_SHOOTING_ZONE)
//                        shooters.setShootingLocation(Shooter.ShootingLocation.NEAR);
//                    else if (robotZone == IMULocalizer.RobotZone.MEDIUM_SHOOTING_ZONE)
//                        shooters.setShootingLocation(Shooter.ShootingLocation.MEDIUM);
//                    else if (robotZone == IMULocalizer.RobotZone.FAR_SHOOTING_ZONE)
//                        shooters.setShootingLocation(Shooter.ShootingLocation.FAR);
//                    else if (robotZone == IMULocalizer.RobotZone.FAR_FAR_SHOOTING_ZONE)
//                        shooters.setShootingLocation(Shooter.ShootingLocation.FAR_FAR);
                }
            }
        }
        else
        {
            shooters.setShootingLocation(Shooter.ShootingLocation.MEDIUM);
        }

        if(isShooterOn)
            shooters.doFlyWheelPID();
        else
            shooters.stop();
    }

    private void doRumble()
    {
        if (gameTimer.getElapsedTimeSeconds() >= 80 && rumbleEndgame == 0)  {
            rumbleEndgame = 1;
            gamepad1.runRumbleEffect(strongRumbleEffect);
            gamepad2.runRumbleEffect(strongRumbleEffect);
        }

        if (gameTimer.getElapsedTimeSeconds() >= 100 && rumbleEndgame == 1)  {
            rumbleEndgame = 2;
            gamepad1.runRumbleEffect(strongRumbleEffect);
            gamepad2.runRumbleEffect(strongRumbleEffect);
        }
    }

    private void initSubsystems()
    {
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


        actionTimer = new Timer();
        gameTimer = new Timer();


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

    }
}

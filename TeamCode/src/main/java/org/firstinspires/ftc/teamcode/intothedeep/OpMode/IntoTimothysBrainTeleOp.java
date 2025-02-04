package org.firstinspires.ftc.teamcode.intothedeep.OpMode;

import android.widget.Button;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SimpleDriveTrain;
import org.firstinspires.ftc.teamcode.common.Helper;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.ClawRotor;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.IntakeSlide;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.Slide;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Into the Brain", group = "A")

public class IntoTimothysBrainTeleOp extends LinearOpMode {

    private Timer actionTimer;
    private Timer gameTimer;
    private SimpleDriveTrain driveTrain;
    private Slide slide;
    private Claw claw;
    private ClawRotor clawRotor;
    private OuttakeArm outtakeArm;
    private Intake intake;
    private IntakeSlide intakeSlide;
    private GamepadEx gamepad1Ex;
    private GamepadEx gamepad2Ex;

    private ToggleButtonReader leftBumper1;
    private TriggerReader leftTrigger2Reader;
    private TriggerReader rightTrigger2Reader;
    private TriggerReader rightTrigger1Reader;
    private TriggerReader leftTrigger1Reader;

    private ButtonReader dpad_up;
    private ButtonReader dpad_down;
    private ButtonReader dpad_right;
    private ButtonReader dpad_left;
    private ButtonReader back;
    private ButtonReader start;

    private Slide.SlideTargetPosition slideOp;
    private boolean robotCentric;
    private boolean leftBumperToggled;
    private boolean goingIn;
    private boolean running;
    private int actionsOrSomething;
    private int rumble;
    private boolean specMode;
    Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence
    Gamepad.RumbleEffect intakeRumbleEffect;

    //TODO
    enum AutoCompleteMode
    {
        MANUAL,
        SAMPLE_DELIVERY_START,
        SAMPLE_DELIVERY_CLAW_CLOSE,
        SAMPLE_DELIVER_SLIDE_UP,
        SAMPLE_DROP_START,
        SAMPLE_DROP_CLAW_CLOSE,
        SAMPLE_DROP_SLIDE_UP,
        SAMPLE_DROP_SLIDE_DOWN,
        SPECIMEN_RESET_START,
        SPECIMEN_RESET_SLIDE_UP,
        SPECIMEN_RESET_ARM_DOWN,
        SPECIMEN_RESET_SLIDE_DOWN,
        RESET_START,
        RESET_ARM_DOWN
    }

    IntoTimothysBrainTeleOp.AutoCompleteMode autoCompleteMode;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initializing drive train");
        telemetry.update();
        driveTrain = new SimpleDriveTrain(hardwareMap, this, false);

        telemetry.addLine("Initializing slide");
        telemetry.update();
        slide = new Slide(hardwareMap, this);
        slide.runWithEncoder();

        telemetry.addLine("Initializing claw");
        telemetry.update();
        claw = new Claw(hardwareMap, this);

        telemetry.addLine("Initializing claw rotor");
        telemetry.update();
        clawRotor = new ClawRotor(hardwareMap, this);


        telemetry.addLine("Initializing outtake arm");
        telemetry.update();
        outtakeArm = new OuttakeArm(hardwareMap, this);

        telemetry.addLine("Initializing intake");
        telemetry.update();
        intake = new Intake(hardwareMap, this);

        telemetry.addLine("Initializing intake slide");
        telemetry.update();
        intakeSlide = new IntakeSlide(hardwareMap);

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        leftBumper1 = new ToggleButtonReader(
                gamepad1Ex, GamepadKeys.Button.LEFT_BUMPER
        );
        leftTrigger1Reader = new TriggerReader(
                gamepad1Ex, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        leftTrigger2Reader = new TriggerReader(
                gamepad2Ex, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        rightTrigger2Reader = new TriggerReader(
                gamepad2Ex, GamepadKeys.Trigger.RIGHT_TRIGGER
        );
        rightTrigger1Reader = new TriggerReader(
                gamepad1Ex, GamepadKeys.Trigger.RIGHT_TRIGGER
        );
        dpad_up = new ButtonReader(
                gamepad1Ex, GamepadKeys.Button.DPAD_UP
        );
        dpad_down = new ButtonReader(
                gamepad1Ex, GamepadKeys.Button.DPAD_DOWN
        );
        dpad_left = new ButtonReader(
                gamepad1Ex, GamepadKeys.Button.DPAD_LEFT
        );
        dpad_right = new ButtonReader(
                gamepad1Ex, GamepadKeys.Button.DPAD_RIGHT
        );
        back = new ButtonReader(
                gamepad1Ex, GamepadKeys.Button.BACK
        );
        start = new ButtonReader(
                gamepad1Ex, GamepadKeys.Button.START
        );

        actionTimer = new Timer();

        gameTimer = new Timer();

        rumble = 0;

        specMode = false;

        running = false;

        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.1, 0.1, 200)  //
                .addStep(1, 1, 1000)
                .addStep(0.1, 0.1, 200)
                .build();

        intakeRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.3, 0.3, 500)  //
                .build();

        //wait for play button clicked
        waitForStart();
        gameTimer.resetTimer();


        //slide is manually controlled
        slideOp = Slide.SlideTargetPosition.MANUAL;

        //auto complete is not enabled yet
        //auto complete is entered when gp2 left trigger,
        //right trigger or left bumper is pressed
        autoCompleteMode = AutoCompleteMode.MANUAL;

        //ready for sample intake
        clawRotor.MoveToSampleIntakePosition();
        outtakeArm.RotateTo(outtakeArm.SPECIMEN_SCORE_POSITION);

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            gamepad1Ex.readButtons();
            gamepad2Ex.readButtons();;

            leftTrigger2Reader.readValue();
            rightTrigger2Reader.readValue();
            rightTrigger1Reader.readValue();
            leftTrigger1Reader.readValue();
            leftBumper1.readValue();

            ///////////////////////////////////////////////
            //driver
            intakeOp();


            ///////////////////////////////////////////////////
            //claw person
            outtakeOp();

            if(rightTrigger1Reader.isDown() && intakeSlide.isTouchSensorPressed())
            {
                //gamepad1.runRumbleEffect(intakeRumbleEffect);
                gamepad2.runRumbleEffect(intakeRumbleEffect);
                goingIn = true;
            }
            else if(leftTrigger1Reader.wasJustReleased()){
                goingIn = false;
            }

            //drive train
            //if(gamepad1.left_bumper)
            driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            //else
            //    driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


            if (gameTimer.getElapsedTimeSeconds() >= 70 && rumble == 0)  {
                rumble = 1;
                gamepad1.runRumbleEffect(customRumbleEffect);
                gamepad2.runRumbleEffect(customRumbleEffect);
            }

            if (gameTimer.getElapsedTimeSeconds() >= 110 && rumble == 1){
                rumble = 2;
                gamepad1.runRumbleEffect(customRumbleEffect);
                gamepad2.runRumbleEffect(customRumbleEffect);
            }

            telemetry.update();
        }
    }

    private void intakeOp()
    {
        //intake.DetectSample();

        //horizontal slide operation
        //double newValue = (value - oldMin) * (newMax - newMin) / (oldMax - oldMin) + newMin;
        double slideOutSpeed = gamepad1Ex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);//gamepad1.left_trigger;
        double slideInSpeed = gamepad1Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);//gamepad1.right_trigger;
        if(slideOutSpeed >= 0.05) {

            //telemetry.addData("left_trigger", slideOutSpeed);

            //scale from [0 1] to [0.5 1], move out, (slideOutSpeed + 1) * 0.5

            //now since squared, the number could be less than 0.5, which will
            //pull the slide back
            slideOutSpeed = Helper.squareWithSign((slideOutSpeed + 1) * 0.5);

            //cap the retraction and push power into the desired range
            if(slideOutSpeed < 0.43)//0.43
                slideOutSpeed = 0.43;

            //if(slideOutSpeed > 0.9)//0.7
            //    slideOutSpeed = 0.9;

            intakeSlide.Move(slideOutSpeed);

            //telemetry.addData("slideOutSpeed", slideOutSpeed);
        }
        else if(slideInSpeed >= 0.05) {

            //when retracting back, move the intake to the outtake position
            //for the claw to pick the sample up
            intake.MoveToOuttakePosition();

            //scale from [0 1] to [0 0.5], move in
            intakeSlide.Move(0.5-slideInSpeed*0.5);

            //telemetry.addData("slideInSpeed", 0.5-slideInSpeed*0.5);
        }
        else
            intakeSlide.Move(0.485);

        //
        if(leftBumper1.getState())
            leftBumperToggled = true;
        else
            leftBumperToggled = false;

        //intake control
        if (gamepad1Ex.isDown(GamepadKeys.Button.RIGHT_BUMPER)) //right bumper take in
            intake.SetIntakeSpinner(Intake.IntakeMode.OUT);
        else {
            if (leftBumperToggled)
                intake.SetIntakeSpinner(Intake.IntakeMode.IN);
            else
                intake.SetIntakeSpinner(Intake.IntakeMode.IDLE);
        }


        //intake sample container
        if(gamepad1Ex.isDown(GamepadKeys.Button.A))//gamepad1.a)
            intake.MoveToOuttakePosition();
        else if(gamepad1Ex.isDown(GamepadKeys.Button.X)) //(gamepad1.x)
            intake.MoveToReadyPosition();
        else if(gamepad1Ex.isDown(GamepadKeys.Button.Y)) //(gamepad1.y)
            intake.MoveToIntakePosition();
        else if(gamepad1Ex.isDown(GamepadKeys.Button.B)) //(gamepad1.b)
            intake.MoveToHeadDownPosition();
    }

    private void outtakeOp()
    {
        if(back.wasJustPressed()){
            specMode = false;
        }
        else if(start.wasJustPressed()){
            specMode = true;
        }

        if(specMode)
        {
            if (dpad_down.wasJustPressed())
        {
            autoCompleteMode = AutoCompleteMode.SPECIMEN_RESET_START;
        }
            else if (dpad_up.wasJustPressed()) //button x, set the arm to pickup specimen from human player
        {
            clawRotor.MoveToSpecimenIntakePosition();
            outtakeArm.RotateTo(outtakeArm.SPECIMEN_PICKUP_POSITION);
        }
            else if(dpad_up.wasJustReleased())
        {
            //TODO: REPLACE SLEEP WITH STATE MACHINE
            claw.close();
            if(actionTimer.getElapsedTime() > 2000) {
                actionTimer.resetTimer();
                outtakeArm.RotateTo(outtakeArm.SPECIMEN_SHUFFLE_POSITION);
                clawRotor.MoveToSampleIntakePosition();
            }
        }
            else if(dpad_right.wasJustPressed()){
                outtakeArm.RotateTo(outtakeArm.SPECIMEN_SCORE_POSITION);
            }
            else if(dpad_left.wasJustPressed())
        {
            claw.open();
        }
        }
        else{
/** auto complete */
            //leftTrigger2: slide up to high basket, claw close, wait 200ms, slide up, wait 100 ms, arm up
            //rightTrigger2: arm down, wait 200 ms, slide down to the bottom
            //left bumper: claw close, wait 200ms, slide up, arm up
            if(dpad_up.wasJustPressed())
            {
                //actionTimer.resetTimer();
                running = true;
                autoCompleteMode = IntoTimothysBrainTeleOp.AutoCompleteMode.SAMPLE_DELIVERY_START;
            }
            else if (dpad_down.wasJustPressed())
            {
                running = true;
                autoCompleteMode = IntoTimothysBrainTeleOp.AutoCompleteMode.RESET_START;
            }

            else if (dpad_up.wasJustReleased())
            {
                claw.open();
            }
        }


        /** outtake arm operation */
        //button a, set the arm to pickup a sample from the intake
        if (gamepad2Ex.isDown(GamepadKeys.Button.A)) {
            outtakeArm.RotateTo(outtakeArm.SAMPLE_PICKUP_POSITION);
            clawRotor.MoveToSampleIntakePosition();
        } else if (gamepad2Ex.isDown(GamepadKeys.Button.Y)) //button y, set the arm to the specimen ready position
        {
            //change to SPECIMEN_READY_POSITION if it doesn't work
            outtakeArm.RotateTo(outtakeArm.SPECIMEN_SHUFFLE_POSITION);

            //old high with momentum
            //outtakeArm.Rotate(outtakeArm.SPECIMEN_READY_POSITION);


            clawRotor.MoveToSampleIntakePosition();
        } else if (gamepad2Ex.isDown(GamepadKeys.Button.B)) //button b, set the arm to score samples into high basket
        {
            outtakeArm.RotateTo(outtakeArm.SPECIMEN_SCORE_POSITION);
            clawRotor.MoveToSampleIntakePosition();
        }







        if(running){
            switch (autoCompleteMode) {
                ////only when in auto complete mode
                ////For sample delivery auto complete
                case SAMPLE_DELIVERY_START:
                    actionTimer.resetTimer();
                    autoCompleteMode = IntoTimothysBrainTeleOp.AutoCompleteMode.SAMPLE_DELIVERY_CLAW_CLOSE;
                    claw.close();
                    break;
                case SAMPLE_DELIVERY_CLAW_CLOSE:
                    if (actionTimer.getElapsedTime() > 200)
                    {
                        actionTimer.resetTimer();
                        autoCompleteMode = IntoTimothysBrainTeleOp.AutoCompleteMode.SAMPLE_DELIVER_SLIDE_UP;
                        if (slideOp != Slide.SlideTargetPosition.HIGH_BASkET) {
                            slideOp = Slide.SlideTargetPosition.HIGH_BASkET;
                            slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.HIGH_BASkET, 1);
                        }
                    }
                    break;
                case SAMPLE_DELIVER_SLIDE_UP:
                    if (actionTimer.getElapsedTime() > 200) {
                        outtakeArm.RotateTo(outtakeArm.SAMPLE_DELIVERY_POSITION);
                        clawRotor.MoveToSampleIntakePosition();
                        autoCompleteMode = IntoTimothysBrainTeleOp.AutoCompleteMode.MANUAL;
                    }
                    running = false;
                    break;
                ////Sample drop auto complete
                case SAMPLE_DROP_START:
                    actionTimer.resetTimer();
                    autoCompleteMode = IntoTimothysBrainTeleOp.AutoCompleteMode.SAMPLE_DROP_CLAW_CLOSE;
                    claw.close();
                    break;
                case SAMPLE_DROP_CLAW_CLOSE:
                    if (actionTimer.getElapsedTime() > 200) {
                        actionTimer.resetTimer();
                        autoCompleteMode = IntoTimothysBrainTeleOp.AutoCompleteMode.SAMPLE_DROP_SLIDE_UP;
                        if (slideOp != Slide.SlideTargetPosition.DROP_SAMPLE) {
                            slideOp = Slide.SlideTargetPosition.DROP_SAMPLE;
                            slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.DROP_SAMPLE, 1);
                        }
                    }
                    break;
                case SAMPLE_DROP_SLIDE_UP:
                    if (actionTimer.getElapsedTime() > 200) {
                        clawRotor.MoveToSpecimenIntakePosition();
                        outtakeArm.RotateTo(outtakeArm.SPECIMEN_PICKUP_POSITION);
                        autoCompleteMode = IntoTimothysBrainTeleOp.AutoCompleteMode.SAMPLE_DROP_SLIDE_DOWN;
                        actionTimer.resetTimer();
                    }
                    break;
                case SAMPLE_DROP_SLIDE_DOWN:
                    if (actionTimer.getElapsedTime() > 150) {
                        if (slideOp != Slide.SlideTargetPosition.DOWN) {
                            slideOp = Slide.SlideTargetPosition.DOWN;
                            slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.DOWN, 1); //ground
                            autoCompleteMode = IntoTimothysBrainTeleOp.AutoCompleteMode.MANUAL;
                        }
                    }
                    break;
                ////reset specimen position to intake position

                case SPECIMEN_RESET_START:
                    actionTimer.resetTimer();
                    autoCompleteMode = AutoCompleteMode.SPECIMEN_RESET_SLIDE_UP;
                    if (slideOp != Slide.SlideTargetPosition.DROP_SAMPLE) {
                        slideOp = Slide.SlideTargetPosition.DROP_SAMPLE;
                        slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.DROP_SAMPLE, 1);
                    }

                    break;

                case SPECIMEN_RESET_SLIDE_UP:
                    if (actionTimer.getElapsedTime() > 200){
                        actionTimer.resetTimer();
                        claw.open();
                        autoCompleteMode = AutoCompleteMode.SPECIMEN_RESET_SLIDE_DOWN;
                        outtakeArm.RotateTo(outtakeArm.SAMPLE_PICKUP_POSITION);
                        clawRotor.MoveToSampleIntakePosition();
                    }
                    break;

                case SPECIMEN_RESET_SLIDE_DOWN:
                    if (actionTimer.getElapsedTime() >= 500){
                        autoCompleteMode = AutoCompleteMode.MANUAL;
                        if(slideOp != Slide.SlideTargetPosition.DOWN) {
                            slideOp = Slide.SlideTargetPosition.DOWN;
                            slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.DOWN, 1); //ground
                        }
                        running = false;
                    }
                    break;

                ////reset slide and claw auto complete
                case RESET_START:
                    outtakeArm.RotateTo(outtakeArm.SAMPLE_PICKUP_POSITION);
                    claw.open();
                    clawRotor.MoveToSampleIntakePosition();
                    autoCompleteMode = IntoTimothysBrainTeleOp.AutoCompleteMode.RESET_ARM_DOWN;
                    actionTimer.resetTimer();
                    break;
                case RESET_ARM_DOWN:
                    if (actionTimer.getElapsedTime() > 150) {
                        if(slideOp != Slide.SlideTargetPosition.DOWN) {
                            slideOp = Slide.SlideTargetPosition.DOWN;
                            slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.DOWN, 1); //ground
                        }
                        autoCompleteMode = IntoTimothysBrainTeleOp.AutoCompleteMode.MANUAL;
                    }
                    running = false;
                    break;
                case MANUAL:
                    break;
            }

            //this is to stop motor if position reached
            slide.autoMoveToWithoutWaitingLoop();
        }
    }
}

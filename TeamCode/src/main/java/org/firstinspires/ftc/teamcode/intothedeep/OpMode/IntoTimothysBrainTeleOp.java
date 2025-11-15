package org.firstinspires.ftc.teamcode.intothedeep.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SimpleDriveTrain;
import org.firstinspires.ftc.teamcode.common.Helper;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.ClawRotor;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.IntakeSlide;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.Slide;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Into Timothy's Brain (TeleOp)", group = "Into the Deep")
@Disabled

public class IntoTimothysBrainTeleOp extends LinearOpMode {

    private Timer actionTimer;
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

    private Slide.SlideTargetPosition slideOp;
    private boolean robotCentric;
    private boolean leftBumperToggled;

    private boolean specMode;

    private int outtakeState;


    //TODO COOK PLEASE add a solo op mode today! (aka porting all the stuff from what's it called gamepad 2 to gamepad 1
    enum AutoCompleteMode
    {
        MANUAL,
        SAMPLE_DELIVERY_START,
        SAMPLE_DELIVER_SLIDE_UP,
        SAMPLE_DROP_START,
        SAMPLE_DROP_SLIDE_UP,
        SAMPLE_DROP_SLIDE_DOWN,
        RESET_START,
        RESET_ARM_DOWN
    }

    AutoCompleteMode autoCompleteMode;

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

        leftTrigger2Reader = new TriggerReader(
                gamepad2Ex, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        rightTrigger2Reader = new TriggerReader(
                gamepad2Ex, GamepadKeys.Trigger.RIGHT_TRIGGER
        );

        actionTimer = new Timer();

        waitForStart();

        //slide is manually controlled
        slideOp = Slide.SlideTargetPosition.MANUAL;
        autoCompleteMode = AutoCompleteMode.MANUAL;
        clawRotor.MoveToSampleIntakePosition();
        outtakeArm.RotateTo(outtakeArm.SPECIMEN_READY_POSITION);
        boolean robotCentric = true;
        boolean leftBumperToggled = false;
        boolean specMode = false;

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            gamepad1Ex.readButtons();
            gamepad2Ex.readButtons();;

            leftTrigger2Reader.readValue();
            rightTrigger2Reader.readValue();

            leftBumper1.readValue();

            /////////////////////////////////////////////////
            //mode switch
            if(gamepad1Ex.isDown(GamepadKeys.Button.START)){
                specMode = true;
            }
            if(gamepad1Ex.isDown(GamepadKeys.Button.BACK)){
                specMode = false;
            }


            ///////////////////////////////////////////////
            //intaking
            intakeOp();



            ///////////////////////////////////////////////////
            //scoring
            outtakeOp();


            //drive train
            //if(gamepad1.left_bumper)
            driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            //else
            //    driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.update();
        }
    }

    private void intakeOp()
    {
        //horizontal slide operation
        //double newValue = (value - oldMin) * (newMax - newMin) / (oldMax - oldMin) + newMin;
        double slideOutSpeed = gamepad1Ex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);//gamepad1.left_trigger;
        double slideInSpeed = gamepad1Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);//gamepad1.right_trigger;
        if(slideOutSpeed >= 0.05) {

            telemetry.addData("left_trigger", slideOutSpeed);

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

            telemetry.addData("slideOutSpeed", slideOutSpeed);
        }
        else if(slideInSpeed >= 0.05) {

            //when retracting back, move the intake to the outtake position
            //for the claw to pick the sample up
            intake.MoveToOuttakePosition();

            //scale from [0 1] to [0 0.5], move in
            intakeSlide.Move(0.5-slideInSpeed*0.5);

            telemetry.addData("slideInSpeed", 0.5-slideInSpeed*0.5);
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
    private void setState(int changedState, int newState){
        changedState = newState;
    }
    private void outtakeOp()
    {
        switch (outtakeState){
            //sample to high basket
            case 1:
                if (actionTimer.getElapsedTime() >= 200){
                    outtakeArm.RotateTo(outtakeArm.SAMPLE_DELIVERY_POSITION);
                    slideOp = Slide.SlideTargetPosition.HIGH_BASkET;
                    clawRotor.MoveToSampleIntakePosition();
                    setState(outtakeState, 3);
                    break;
                }
            //specimen ready
            case 2:
                if (actionTimer.getElapsedTime() >= 200) {
                    outtakeArm.RotateTo(outtakeArm.SPECIMEN_SHUFFLE_POSITION);
                    clawRotor.MoveToSampleIntakePosition();
                    setState(outtakeState, 3);
                    break;
                }
            case 3:
                break;
                }
        //vertical slide operation
        //By holding the left bumper, manual operation
        //if (gamepad2Ex.isDown(GamepadKeys.Button.LEFT_BUMPER)){//gamepad2.left_bumper) {
        if(specMode)
        {
            if (gamepad1Ex.isDown(GamepadKeys.Button.DPAD_DOWN)){
                outtakeArm.RotateTo(outtakeArm.SPECIMEN_PICKUP_POSITION);
                actionTimer.resetTimer();
            }
            if (gamepad1Ex.isDown(GamepadKeys.Button.DPAD_LEFT)){
                claw.open();
            }
            else if (gamepad1Ex.isDown(GamepadKeys.Button.DPAD_UP)) {//gamepad2.dpad_up
                claw.close();
                if (actionTimer.getElapsedTime() > 200){
                    actionTimer.resetTimer();
                    setState(outtakeState,2);
                }
            }
            else if (gamepad1Ex.isDown(GamepadKeys.Button.DPAD_RIGHT)) { //gamepad2.dpad_left
                outtakeArm.RotateTo(outtakeArm.SPECIMEN_SCORE_POSITION);
                clawRotor.MoveToSampleIntakePosition();
            }
            //this is to stop motor if position reached
            slide.autoMoveToWithoutWaitingLoop();
        }
        else {
            if (gamepad1Ex.isDown(GamepadKeys.Button.DPAD_DOWN) && slideOp != Slide.SlideTargetPosition.DOWN){
                outtakeArm.RotateTo(outtakeArm.SAMPLE_PICKUP_POSITION);
                slideOp = Slide.SlideTargetPosition.DOWN;
            }
            else if (gamepad1Ex.isDown(GamepadKeys.Button.DPAD_UP) && slideOp != Slide.SlideTargetPosition.HIGH_BASkET) {//gamepad2.dpad_up
                claw.close();
                if (actionTimer.getElapsedTime() > 200) {
                    actionTimer.resetTimer();
                    setState(outtakeState, 1);
                }
            }
            else{
                claw.open();
            }
            //this is to stop motor if position reached
            slide.autoMoveToWithoutWaitingLoop();
        }

        //claw operation
        //boolean currentBumperState  = gamepad2.right_bumper;
        //hold right bumper to close the claw


        //only when in auto complete mode

    }
}

package org.firstinspires.ftc.teamcode.biobuzz.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake;

//@Disabled

@TeleOp(name = "Intake Test", group = "BioBuzz Test")
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException
    {
        Intake intake = new Intake(hardwareMap, this);

        telemetry.addData("gamepad1.a", "intake/transfer");
        telemetry.addData("gamepad1.b", "intake");
        telemetry.addData("gamepad1.y", "feed");
        telemetry.addData("gamepad1.x", "spitout");
        telemetry.addData("gamepad1.left_stick_y up", "outtake");
        telemetry.addData("gamepad1.left_stick_y down", "intake");

        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;

        while(!isStopRequested() && opModeIsActive())
        {
            if(gamepad1.right_bumper) {
//                if (intake.detectedArtifacts() == 3) {
//                    //intake.setIntakeMode(Intake.IntakeMode.IDLE);
//                    //make the intake motor slower but don't stop it
//                    intake.intake(0.3, 0);//0
//                    intake.setLedColor(Intake.LED_GREEN);
//                }
//                else if (intake.detectedArtifacts() == 2) {
//                    intake.setLedColor(Intake.LED_YELLOW);
//                    intake.setIntakeMode(Intake.IntakeMode.HIN);
//                }
//                else if (intake.detectedArtifacts() == 1){
//                    intake.setLedColor(Intake.LED_ORANGE);
//                    intake.intake(0.95,0.925);
//                }
//                else
//                {
//                    intake.setLedColor(Intake.LED_OFF);
//                    intake.intake(0.95,0.925);//0.925
//                }
            }
            else if(gamepad1.a)
                intake.setIntakeMode(Intake.IntakeMode.IN);
                //intake.intake(0.95, 0.95);//0.95
            else if(gamepad1.b)
                intake.setIntakeMode(Intake.IntakeMode.INTAKE_ONLY);
            else if(gamepad1.y)
                intake.setIntakeMode(Intake.IntakeMode.FEED);
            else if(gamepad1.x)
                intake.setIntakeMode(Intake.IntakeMode.OUT);
            else
            {
                double intakePower = -gamepad1.left_stick_y;
                if (intakePower >= 0) {
                    intake.outtake(intakePower);
                } else {
                    intake.intake(intakePower);
                }

                telemetry.addData("Power", intakePower);
            }

            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.decode.Test;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.Subsystems.Intake;

@TeleOp(name = "Intake Test", group = "Decode Test")
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException
    {
        Intake intake = new Intake(hardwareMap, this);

        telemetry.addData("left_stick_y up", "outtake");
        telemetry.addData("left_stick_y down", "intake");

        int[] artifactColors; //0 - top, 1 - middle, 2 - bottom

        artifactColors = new int[3];
        artifactColors[0] = artifactColors[1] = artifactColors[2] = Color.WHITE;

        waitForStart();

        if(isStopRequested()) return;

        while(!isStopRequested() && opModeIsActive())
        {
            if(gamepad1.right_bumper) {
                if (artifactColors[0] != Color.WHITE &&
                        artifactColors[1] != Color.WHITE &&
                        artifactColors[2] != Color.WHITE)
                    intake.setIntakeMode(Intake.IntakeMode.IDLE);
                else if (artifactColors[0] != Color.WHITE &&
                        artifactColors[1] != Color.WHITE)
                    intake.setIntakeMode(Intake.IntakeMode.HIN);
                else
                    intake.intake(0.95);;
            }
            else if(gamepad1.a)
                //intake.setIntakeMode(Intake.IntakeMode.IN);
                intake.intake(0.95);
            else if(gamepad1.b)
                intake.setIntakeMode(Intake.IntakeMode.HIN);
            else if(gamepad1.y)
                intake.setIntakeMode(Intake.IntakeMode.FEED);
            else if(gamepad1.x)
                intake.setIntakeMode(Intake.IntakeMode.OUT);
            else
            {
                double intakePower = -gamepad1.left_stick_y;
                if (intakePower >= 0) {
                    intake.intake(intakePower);
                } else {
                    intake.outtake(intakePower);
                }

                telemetry.addData("Power", intakePower);
            }

            artifactColors = intake.detectArtifactColors();

            telemetry.addData("Artifact Top", colorName(artifactColors[0]));
            telemetry.addData("Artifact Middle", colorName(artifactColors[1]));
            telemetry.addData("Artifact Bottom", colorName(artifactColors[2]));


            telemetry.update();
        }
    }

    String colorName(int color)
    {
        if (color == Color.GREEN)
            return "Green";
        else if (color == Color.BLUE)
            return "Purple";
        else
            return "No Found";
    }
}

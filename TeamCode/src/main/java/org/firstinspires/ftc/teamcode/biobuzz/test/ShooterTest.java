package org.firstinspires.ftc.teamcode.biobuzz.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Shooter;

//@Disabled


@TeleOp(name = "Pollen Shooter Test", group = "BioBuzz Test")

public class ShooterTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Shooter pollenShooter = new Shooter(hardwareMap,
                this,
                "pollenShooter",
                0.035,
                0.1,
                0,
                0.9,
                0.55,
                0.52,
                0.48,
                0.45);


        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.right_bumper) {

            }
        }
    }
}

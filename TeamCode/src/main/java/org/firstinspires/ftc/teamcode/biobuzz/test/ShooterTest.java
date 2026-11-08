package org.firstinspires.ftc.teamcode.biobuzz.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.subsystems.PollenShooter;

//@Disabled


@TeleOp(name = "Pollen Shooter Test", group = "BioBuzz Test")

public class ShooterTest extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        PollenShooter shooter = new PollenShooter(hardwareMap, this, 1);


        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.right_bumper) {

            }
        }
    }
}

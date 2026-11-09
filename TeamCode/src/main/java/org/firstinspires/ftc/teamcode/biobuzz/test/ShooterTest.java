package org.firstinspires.ftc.teamcode.biobuzz.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Shooter;

import java.util.List;

//@Disabled


@TeleOp(name = "Pollen Shooter Test", group = "BioBuzz Test")

public class ShooterTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Shooter pollenShooter = new Shooter(hardwareMap,
                this,
                "pollenFlywheel",
                0.035,
                0.1,
                0,
                0.9,
                0.55,
                0.52,
                0.50,
                0.48);


        telemetry.update();

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

        if (isStopRequested()) return;

        pollenShooter.setPower(0.95);
        sleep(200);//1000

        ///let the PID work for a while
        for (int i = 0; i < 30; i++) {
            pollenShooter.doFlyWheelPID();
            sleep(20);
        }

        while (!isStopRequested() && opModeIsActive()) {

            hubs.forEach(LynxModule::clearBulkCache);

            pollenShooter.doFlyWheelPID();

            telemetry.update();
        }
    }
}

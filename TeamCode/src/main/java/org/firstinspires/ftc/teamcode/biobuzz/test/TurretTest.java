package org.firstinspires.ftc.teamcode.biobuzz.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.biobuzz.opmode.BioBuzzDashboard;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Turret;

import java.util.List;

@TeleOp(name = "Turret Test", group = "BioBuzz Test")
public class TurretTest  extends LinearOpMode {

    private boolean isBlueTeleOp = true;
    Pose2D robotPose;

    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //waitForStart();
        while (!isStarted() && !isStopRequested()) {

            if(gamepad1.a) {
                isBlueTeleOp = false;
            }
            else if (gamepad1.b) {
                isBlueTeleOp = true;
            }

            if(isBlueTeleOp) {
                telemetry.addLine("Alliance Selected: BLUE BLUE BLUE");

                //Blue near auto starting pos, keep the same as blue near auto
                if(robotPose == null)
                    robotPose = BioBuzzDashboard.BLUE_IMU_RESET_POSE;
            }
            else {
                telemetry.addLine("Alliance Selected: RED RED RED");

                //Red near auto starting pos, keep the same as red near auto
                if (robotPose == null)
                    robotPose = BioBuzzDashboard.RED_IMU_RESET_POSE;
            }

            telemetry.addLine("");
            telemetry.addLine("WARNING: Select the right allaince!!!");
            telemetry.addLine("Gamepad1.A: Alliance RED");
            telemetry.addLine("Gamepad1.B: Alliance BLUE");
            telemetry.addLine("-----------------------");
            telemetry.addData("Auto end X (Inch):", robotPose.getX(DistanceUnit.INCH));
            telemetry.addData("Auto end Y (Inch):", robotPose.getY(DistanceUnit.INCH));
            telemetry.addData("Auto end Heading (Degree) :", robotPose.getHeading(AngleUnit.DEGREES));

            telemetry.update();
        }

        int alliance;

        if(isBlueTeleOp)
            alliance = BioBuzzDashboard.ALLIANCE_BLUE;
        else
            alliance = BioBuzzDashboard.ALLIANCE_RED;

        Turret turret = new Turret(hardwareMap, this, robotPose,
                alliance, false, false);
        turret.setServoPosition(Turret.CENTER_SERVO_POSITION);

        if(isBlueTeleOp)
            telemetry.addLine("Alliance Selected: BLUE BLUE BLUE");
        else
            telemetry.addLine("Alliance Selected: RED RED RED");

        telemetry.update();

        if(isStopRequested()) return;

        sleep(1000);
        boolean isInitialPinpointPositionSet = false;

        while (!isStopRequested() && opModeIsActive()) {

            if (!isInitialPinpointPositionSet) {
                turret.setIMUPoseToRobotStartPose();
                isInitialPinpointPositionSet = true;
            }

            hubs.forEach(LynxModule::clearBulkCache);

            turret.autoAim(true);

            telemetry.update();
        }

    }
}

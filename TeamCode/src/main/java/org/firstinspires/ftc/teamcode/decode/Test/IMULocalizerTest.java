package org.firstinspires.ftc.teamcode.decode.Test;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.decode.OpMode.DecodeBlackBoard;
import org.firstinspires.ftc.teamcode.decode.Subsystems.IMULocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "IMULocalizer Test", group = "Decode Test")
public class IMULocalizerTest extends LinearOpMode {

    //update in real-time
    private boolean isBlueTeleOp = true;
    Pose2D robotPose;
    IMULocalizer localizer;
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {


        follower = Constants.createFollower(hardwareMap);

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
                telemetry.addLine("Localizer Selected: BLUE BLUE BLUE");

                //Blue near auto starting pos, keep the same as blue near auto
                if(robotPose == null)
                    robotPose = DecodeBlackBoard.BLUE_NEAR_START_POSE;
            }
            else {
                telemetry.addLine("Localizer Selected: RED RED RED");

                //Red near auto starting pos, keep the same as red near auto
                if (robotPose == null)
                    robotPose = DecodeBlackBoard.RED_NEAR_START_POSE;
            }

            telemetry.addLine("");
            telemetry.addLine("WARNING: Select the right Localizer!!!");
            telemetry.addLine("Gamepad1.A: Localizer RED");
            telemetry.addLine("Gamepad1.B: Localizer BLUE");
            telemetry.addLine("-----------------------");
            telemetry.addData("Auto end X (Inch):", robotPose.getX(DistanceUnit.INCH));
            telemetry.addData("Auto end Y (Inch):", robotPose.getY(DistanceUnit.INCH));
            telemetry.addData("Auto end Heading (Degree) :", robotPose.getHeading(AngleUnit.DEGREES));

            telemetry.update();
        }

        int alliance;
        if(isBlueTeleOp) {
            alliance = DecodeBlackBoard.BLUE;

            localizer = new IMULocalizer(hardwareMap, this, follower, DecodeBlackBoard.BLUE_NEAR_START_POSE,
                    DecodeBlackBoard.BLUE_TARGET_POSE, alliance);
        }
        else {
            alliance = DecodeBlackBoard.RED;

            localizer = new IMULocalizer(hardwareMap, this, follower, DecodeBlackBoard.RED_NEAR_START_POSE,
                    DecodeBlackBoard.RED_TARGET_POSE, alliance);
        }


        if(isBlueTeleOp)
            telemetry.addLine("Alliance Selected: BLUE BLUE BLUE");
        else
            telemetry.addLine("Alliance Selected: RED RED RED");

        telemetry.update();

        if(isStopRequested()) return;

//        sleep(1000);
//
//        boolean isInitialPinpointPositionSet = false;

        while(!isStopRequested() && opModeIsActive()) {

//            if (!isInitialPinpointPositionSet) {
//                localizer.setIMUPoseToRobotStartPose();
//                isInitialPinpointPositionSet = true;
//            }

            hubs.forEach(LynxModule::clearBulkCache);

            boolean isHeadingToGoal = localizer.isHeadingToGoal();
            IMULocalizer.RobotZone robotZone = localizer.getRobotZone();

            double x = localizer.getX();
            double y = localizer.getY();
            double heading = localizer.getHeading();
            double distanceToGoal = localizer.getRobotDistanceToGoal();

            localizer.update();

            telemetry.addData("IMU Heading", "%.3f",  heading);
            telemetry.addData("IMU x", "%.3f", x);
            telemetry.addData("IMU y", "%.3f", y);

            if(isHeadingToGoal)
                telemetry.addLine("Heading to goal: Yes");
            else
                telemetry.addLine("Heading to goal: No");

            telemetry.addData("Distance to goal", "%.3f", distanceToGoal);
            telemetry.addData("Robot Zone", robotZone.ordinal());

            telemetry.update();
        }
    }
}

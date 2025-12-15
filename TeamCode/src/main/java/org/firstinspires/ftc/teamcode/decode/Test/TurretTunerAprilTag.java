package org.firstinspires.ftc.teamcode.decode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@Config
@TeleOp(name = "Concept: Turret Tuner April Tag", group = "Concept")
public class TurretTunerAprilTag extends LinearOpMode {

    private Limelight3A limelight;
    GoBildaPinpointDriver pinpoint;
    private Servo turretLeft, turretRight;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    PIDController controller;

    public static double kP = 0.01; //0.8
    public static double kI = 0; //0.01
    public static double kD = 0;

    public double targetAngleDegree = 0;//-45;
    public double allowedTargetRangeDegree = 55;

    private int targetTagID = 24;


    public void runOpMode() {

        controller = new PIDController(kP, kI, kD);

        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        //turretRight = hardwareMap.get(Servo.class, "turretRight");

        turretLeft.setDirection(Servo.Direction.REVERSE);


        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();
        //pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();


        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {



            // if(gamepad1.a) {

            hubs.forEach(LynxModule::clearBulkCache);

            controller.setPID(kP, kI, kD);

            //pinpoint.update();
            //Pose2D pose2D = pinpoint.getPosition();

            double currentAngle = 0;// = pose2D.getHeading(AngleUnit.DEGREES);
            double pid = 0;

            //if(currentAngle >= targetAngleDegree - allowedTargetRangeDegree &&
            //        currentAngle <= targetAngleDegree + allowedTargetRangeDegree) {

                //turretLeft.setPosition(pid);
            //}

            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {

                currentAngle = result.getTx();



                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {

                    int tagID = fr.getFiducialId();
                    currentAngle = result.getTx();

                    if(tagID == targetTagID) {

                        pid = controller.calculate(currentAngle, 0);
                        turretLeft.setPosition(pid + 0.5);

                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }
                }
            }
            else
                turretLeft.setPosition(0.5);


            telemetry.addData("Camera heading", currentAngle);
            telemetry.addData("Target heading", targetAngleDegree);

            telemetry.addData("PID", pid);
            telemetry.update();

        }
    }

    public void configurePinpoint(){
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }

}

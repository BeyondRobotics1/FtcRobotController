package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class SleeveDetector {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.4;//0.166;
    final int LEFT = 1;
    final int MIDDLE = 2;
    final int RIGHT = 3;
    int location = LEFT;

    AprilTagDetection tagOfInterest = null;

    LinearOpMode mode;

    public SleeveDetector(HardwareMap hardwareMap, LinearOpMode mode)
    {

        this.mode = mode;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public int detectPosition() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {
            boolean tagFound = false;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == 6) {
                    location = LEFT;
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                } else if (tag.id == 16) {
                    location = MIDDLE;
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                } else if (tag.id == 2) {
                    location = RIGHT;
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if (tagFound) {
                mode.telemetry.addLine("Tag of interest is in sight!");
                tagToTelemetry(tagOfInterest);
            } else {
                mode.telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    mode.telemetry.addLine("(The tag has never been seen)");
                } else {
                    mode.telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

        } else {
            mode.telemetry.addLine("Don't see tag of interest :(");

            if (tagOfInterest == null) {
                mode.telemetry.addLine("(The tag has never been seen)");
            } else {
                mode.telemetry.addLine("But we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }

        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

//        /* Update the telemetry */
//        if (tagOfInterest != null) {
//            mode.telemetry.addLine("Tag snapshot:\n");
//            tagToTelemetry(tagOfInterest);
//        } else {
//            mode.telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
//        }

        return location;
    }


    void tagToTelemetry(AprilTagDetection detection)
    {
        mode.telemetry.addLine(String.format("\nDetected April Tag ID=%d", detection.id));


        /*
        mode.telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        mode.telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        mode.telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        mode.telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        mode.telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        mode.telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        */
    }
}





package org.firstinspires.ftc.teamcode.centerstage.picasso;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Red Left", group="Picasso")
//@Disabled

public class CCAutoRedLeft extends LinearOpMode {

    OpenCvWebcam webcam;
    TeamPropDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {

        //
        telemetry.addLine("Initializing drive train");
        telemetry.update();

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //
        telemetry.addLine("Initializing camera");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        pipeline = new TeamPropDeterminationPipeline(TeamPropDeterminationPipeline.TeamPropColor.RED, this);
        webcam.setPipeline(pipeline);

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        TeamPropDeterminationPipeline.TeamPropPosition position = TeamPropDeterminationPipeline.TeamPropPosition.CENTER;

        while (!isStarted() && !isStopRequested()) {

            position = pipeline.getAnalysis();
            telemetry.addData("Position Detected", position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(100);
        }

        //locking the pixel
        PixelPlacer pixelPlacer = new PixelPlacer(hardwareMap, this);


        if (isStopRequested()) return;

        //sleep for other team to finish the auto
        //avoid collision
        if(position == TeamPropDeterminationPipeline.TeamPropPosition.RIGHT)
            sleep(4000);
        else
            sleep(6000);

        PicassoSlide slide = new PicassoSlide(hardwareMap, this);
        slide.runWithEncoder();

        Outtake outtake = new Outtake(hardwareMap, this);
        Arm arm = new Arm(hardwareMap, this);

        //TEST ONLY
        //position = TeamPropDeterminationPipeline.TeamPropPosition.LEFT;
        switch (position)
        {
            case LEFT:
            {
                //move to the left mark position in a straight line
                Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                        .lineToLinearHeading(new Pose2d(38,13))
                        .build();
                drive.followTrajectory(trajectory1);
                sleep(100);

                //unlock the pixel
                pixelPlacer.Unlock();
                sleep(200);

                //move forward
                Trajectory trajectory22 = drive.trajectoryBuilder(trajectory1.end())
                        .forward(15)
                        .build();
                drive.followTrajectory(trajectory22);
                sleep(100);

                //turn left
                drive.turn(Math.toRadians(93));
                sleep(100);

                //move backward
                Trajectory trajectory34 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .back(60)
                        .build();
                drive.followTrajectory(trajectory34);
                sleep(100);

                //move to the left side of the backdrop
                Trajectory trajectory3 = drive.trajectoryBuilder(trajectory34.end())
                        .lineToLinearHeading(new Pose2d(32,-80.5, Math.toRadians(90)))//31.5, -80.5
                        .build();
                drive.followTrajectory(trajectory3);
                //sleep(100);

                placePixelAndPark(drive, slide, outtake, arm, 21);

                break;
            }

            case CENTER:
            {
                //move the center mark position
                Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                        .forward(42.5)
                        .build();
                drive.followTrajectory(trajectory1);

                //unlock the pixel
                pixelPlacer.Unlock();
                sleep(200);

                //move forward
                Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                        .forward(9)//10
                        .build();
                drive.followTrajectory(trajectory2);
                sleep(100);

                //turn left a little bit more than 90 degrees to avoid
                //colliding into the structure
                drive.turn(Math.toRadians(96));
                sleep(100);

                //move back
                Trajectory trajectory22 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .back(55)
                        .build();
                drive.followTrajectory(trajectory22);
                sleep(100);

                //move to the center position of the backdrop
                Trajectory trajectory3 = drive.trajectoryBuilder(trajectory22.end())
                        .lineToLinearHeading(new Pose2d(26.5,-80.5, Math.toRadians(90)))//26
                        .build();
                drive.followTrajectory(trajectory3);
                //sleep(100);


                placePixelAndPark(drive, slide, outtake, arm, 27);

                break;
            }

            case RIGHT:
            {
                //move the right mark position
                Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                        .forward(28)
                        .build();

                drive.followTrajectory(trajectory1);
                sleep(100);

                drive.turn(Math.toRadians(92));//93
                sleep(100);

                Trajectory trajectory2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .back(8)
                        .build();
                drive.followTrajectory(trajectory2);

                //unlock the pixel
                pixelPlacer.Unlock();
                sleep(200);


                //move forward
                Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                        .forward(10)
                        .build();
                drive.followTrajectory(trajectory3);
                sleep(100);

                Trajectory trajectory33 = drive.trajectoryBuilder(trajectory3.end())
                        .strafeRight(24) //24
                        .build();
                drive.followTrajectory(trajectory33);
                sleep(100);

                //move back
                Trajectory trajectory34 = drive.trajectoryBuilder(trajectory33.end())
                        .back(63)//60
                        .build();
                drive.followTrajectory(trajectory34);
                sleep(100);

                //move to the right side of the backdrop
                Trajectory trajectory4 = drive.trajectoryBuilder(trajectory34.end())
                        .lineToLinearHeading(new Pose2d(13,-79.5, Math.toRadians(90)))//12.5, -79.5
                        .build();
                drive.followTrajectory(trajectory4);
                //sleep(100);

                placePixelAndPark(drive, slide, outtake, arm, 34);

                break;
            }
        }

        sleep(1000);

    }

    private void placePixelAndPark(SampleMecanumDrive drive,
                                   PicassoSlide slide,
                                   Outtake outtake,
                                   Arm arm,
                                   double strafeRightInches
    )
    {
        //move slide up to two pixel high
        slide.moveToWithoutWaiting(10.5, 1); //7.3
        arm.goUp();
        sleep(1000);

        //move close to the backdrop
        Trajectory trajectory4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(8.5)
                .build();
        drive.followTrajectory(trajectory4);
        sleep(100);

        //outtake the pixel
        outtake.TakeOut(0.65);
        sleep(900);
        outtake.Hold();
        sleep(100);

        //move away from the backdrop
        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .forward(8)
                .build();
        drive.followTrajectory(trajectory5);
        sleep(100);

        //move the arm and slide down
        //arm.goDown();
        slide.moveToWhiteStripWithoutWaiting(0, 1); //low
        sleep(100);

        //move to the right side of backstage
        Trajectory tra = drive.trajectoryBuilder(trajectory5.end())
                .strafeRight(strafeRightInches)
                .build();
        drive.followTrajectory(tra);

    }
}

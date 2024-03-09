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

@Autonomous(name="Blue Right Right", group="Picasso")
//@Disabled

public class CCAutoBlueRight extends LinearOpMode {
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
        pipeline = new TeamPropDeterminationPipeline(TeamPropDeterminationPipeline.TeamPropColor.BLUE, this);
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

        double satRectLeft = 0;
        double satRectMiddle = 0;
        double satRectRight = 0;

        long delaySeconds = 10;
        while (!isStarted() && !isStopRequested()) {

            if (gamepad1.dpad_down)
                delaySeconds = 0;
            else  if (gamepad1.dpad_left)
                delaySeconds = 4;
            else  if (gamepad1.dpad_up)
                delaySeconds = 8;
            else  if (gamepad1.dpad_right)
                delaySeconds = 10;

            position = pipeline.getAnalysis();
            satRectLeft = pipeline.getSatRectLeft();
            satRectMiddle = pipeline.getSatRectMiddle();
            satRectRight = pipeline.getSatRectRight();

            telemetry.addData("Auto Blue Right. Delayed seconds", "%d", delaySeconds);
            telemetry.addLine("d-down 0, d-left 4, d-up 8, d-right 10");
            telemetry.addData("L", "%.2f", satRectLeft);
            telemetry.addData("C", "%.2f", satRectMiddle);
            telemetry.addData("R", "%.2f", satRectRight);

            telemetry.addData("Realtime analysis", position);
            telemetry.update();

            //telemetry.addData("Auto Blue Left: Position Detected", position);
            //telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(100);
        }

        //locking the pixel
        PixelPlacer pixelPlacer = new PixelPlacer(hardwareMap, this);


        if (isStopRequested()) return;

        //sleep for other team to finish the auto
        //avoid collision
            sleep(delaySeconds * 1000);


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
                //move the left mark position
                Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                        .forward(28)
                        .build();
                drive.followTrajectory(trajectory1);
                sleep(100);

                drive.turn(Math.toRadians(-93));
                sleep(100);
//                drive.adjustHeading(-90);
//                sleep(100);

                Trajectory trajectory2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .back(7)
                        .build();
                drive.followTrajectory(trajectory2);

                //unlock the pixel
                pixelPlacer.Unlock();
                sleep(100);


                //move forward
                Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                        .forward(10)
                        .build();
                drive.followTrajectory(trajectory3);
                sleep(100);


                Trajectory trajectory33 = drive.trajectoryBuilder(trajectory3.end())
                        .strafeLeft(24)//24
                        .build();
                drive.followTrajectory(trajectory33);
                sleep(100);

                Trajectory trajectory34 = drive.trajectoryBuilder(trajectory33.end())
                        .back(62)//62
                        .build();
                drive.followTrajectory(trajectory34);
                sleep(100);

                placePixelAndPark(drive, slide, outtake, arm,15,78,-90, 32);//14, 78, 33, -90

                break;
            }

            case CENTER:
            {
                //move the center mark position
                Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                        .forward(42)//43
                        .build();
                drive.followTrajectory(trajectory1);

                //unlock the pixel
                pixelPlacer.Unlock();
                sleep(200);

                //move forward
                Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                        .forward(10)//10
                        .build();

                drive.followTrajectory(trajectory2);
                sleep(100);

                drive.turn(Math.toRadians(-96));//-94
                Trajectory trajectory22 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .back(60)
                        .build();

                drive.followTrajectory(trajectory22);


                placePixelAndPark(drive, slide, outtake, arm,26.5, 79, -93, 25);//26.5, 78, -90

                break;
            }

            case RIGHT:
            {
                //move the right mark position in a straight line
                Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                        .lineToLinearHeading(new Pose2d(38,-13.5)) //38, -14
                        .build();
                drive.followTrajectory(trajectory1);
                sleep(100);

                //unlock the pixel
                pixelPlacer.Unlock();
                sleep(200);

                //move forward
                Trajectory trajectory22 = drive.trajectoryBuilder(trajectory1.end())
                        .forward(14) //15
                        .build();
                drive.followTrajectory(trajectory22);
                sleep(100);

                //turn right a little bit more than 90 degrees to avoid
                //colliding the structure
                drive.turn(Math.toRadians(-97));//-94
                sleep(100);

                //move backward
                Trajectory trajectory34 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .back(60)
                        .build();
                drive.followTrajectory(trajectory34);


                //move to the right position of the backdrop
                placePixelAndPark(drive, slide, outtake, arm,32, 79.5, -93, 20);//33, 80, -90

                break;
            }
        }

        sleep(1000);
    }

    private void placePixelAndPark(SampleMecanumDrive drive,
                                   PicassoSlide slide,
                                   Outtake outtake,
                                   Arm arm,
                                   double x,
                                   double y,
                                   double heading,
                                   double strafeLeftInches
    )
    {
        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(x,y, Math.toRadians(heading)))//26.5, 79.5, -90 for center
                .build();
        drive.followTrajectory(trajectory);
        //move slide up to two pixels high
        slide.moveToWithoutWaiting(10.5, 1);
        arm.goUp();

        //move close to the backdrop
        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory.end())
                .back(8.5) //8.5
                .build();
        drive.followTrajectory(trajectory4);
        sleep(100);

        //outtake the pixel
        outtake.TakeOut(0.65);
        sleep(800);
        outtake.Hold();
        sleep(100);

        //move away from the backdrop
        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .forward(7) //8
                .build();
        drive.followTrajectory(trajectory5);
        sleep(100);

        //move the arm and slide down
        //arm.goDown();
        slide.moveToWhiteStripWithoutWaiting(0, 1); //low
        sleep(100);

        //move to the right side of backstage
        Trajectory tra = drive.trajectoryBuilder(trajectory5.end())
                .strafeLeft(strafeLeftInches)
                .build();
        drive.followTrajectory(tra);

    }
}

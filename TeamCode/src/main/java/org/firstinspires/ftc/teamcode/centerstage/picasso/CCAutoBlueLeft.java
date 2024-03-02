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

@Autonomous(name="Blue Left", group="Picasso")
//@Disabled

public class CCAutoBlueLeft extends LinearOpMode {

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

        Boolean plus2 = true;
        while (!isStarted() && !isStopRequested()) {

            if(gamepad1.y)
                plus2 = true;
            else if(gamepad1.x)
                plus2 = false;


            position = pipeline.getAnalysis();
            satRectLeft = pipeline.getSatRectLeft();
            satRectMiddle = pipeline.getSatRectMiddle();
            satRectRight = pipeline.getSatRectRight();

            telemetry.addData("Auto Blue Left. Plus 2", "%s", plus2?" Enabled":"Disabled");
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

        drive.resetIMU();

        Intake intake = new Intake(hardwareMap, this);

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
                        .lineToLinearHeading(new Pose2d(32,13.5))//32, 13.5
                        .build();
                drive.followTrajectory(trajectory1);

                //unlock the pixel
                pixelPlacer.Unlock();
                sleep(50);//200

                //move forward
                Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                        .forward(15.5)//15
                        .build();
                drive.followTrajectory(trajectory2);
                //sleep(50);

                double strafeRightInches = 20;//
                if(plus2)
                    strafeRightInches = 0;

                placePixelAndPark(drive, slide, outtake, arm, 7.7,18.5, 32.5, -90, strafeRightInches);//18, 32, -90

                //test
                if(plus2)
                {
                    plusLeftRoute(drive, slide, intake, outtake, arm);

                    //center backdrop
                    placePixelAndPark(drive, slide, outtake, arm, 13, 25, 33, -90, 0); //24.5, 32, -90
                }

                break;
            }

            case CENTER:
            {
                //move the center mark position
                Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                        .forward(41.5)
                        .build();
                drive.followTrajectory(trajectory1);

                //unlock the pixel
                pixelPlacer.Unlock();
                sleep(35);//200

                //move forward
                Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                        .forward(7.5)//5.5
                        .build();
                drive.followTrajectory(trajectory2);

                Trajectory tra = drive.trajectoryBuilder(trajectory2.end())
                        .strafeLeft(15)
                        .build();
                drive.followTrajectory(tra);

                double strafeRightInches = 26;
                if(plus2)
                    strafeRightInches = 0;

                placePixelAndPark(drive, slide, outtake, arm, 7.7,26,32.5,-90, strafeRightInches); //24.5, 32, -90

                //test
                if(plus2) {
                    plusCenterRoute(drive, slide, intake, outtake, arm);

                    //on left backdrop
                    placePixelAndPark(drive, slide, outtake, arm, 13, 20, 32.5, -90, 0);//18, 32, -90
                }

                break;
            }

            case RIGHT:
            {
                //move the RIGHT mark position
                Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                        .forward(26) //26.5
                        .build();
                drive.followTrajectory(trajectory1);
                //sleep(100);

                drive.turn(Math.toRadians(94));//92

                Trajectory trajectory2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .back(6.5)//7
                        .build();
                drive.followTrajectory(trajectory2);

                //unlock the pixel
                pixelPlacer.Unlock();
                sleep(50);

                //move to the right side of the backdrop
                Trajectory trajectory22 = drive.trajectoryBuilder(trajectory2.end())
                        .forward(7)
                        .build();
                drive.followTrajectory(trajectory22);
                //sleep(100);

                double strafeRightInches = 30;
                if(plus2)
                    strafeRightInches = 0;

                placePixelAndPark(drive, slide, outtake, arm, 7.7,35, 32.2, -90, strafeRightInches);//35, 32, -90

                //test
                if(plus2) {
                    plusLeftRoute(drive, slide, intake, outtake, arm);

                    //on left side
                    placePixelAndPark(drive, slide, outtake, arm, 13, 24, 32.5, -90, 0);//18, 32, -90
                }

                break;
            }

        }

        //sleep(1000);
    }

    private void placePixelAndPark(SampleMecanumDrive drive,
                                   PicassoSlide slide,
                                   Outtake outtake,
                                   Arm arm,
                                   double slideHeight,
                                   double x,
                                   double y,
                                   double heading,
                                   double strafeRightInches
                                   ) {
        //move slide up to one pixel high
        arm.goUp();
        slide.moveToWithoutWaiting(slideHeight, 1); //7.4

        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(heading)))
                .build();
        drive.followTrajectory(trajectory);

        //move close to the backdrop
        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory.end())
                .back(8.5)
                .build();
        drive.followTrajectory(trajectory4);

        //outtake the pixel
        outtake.TakeOut(0.65);
        sleep(500);//800
        outtake.Hold();
        //sleep(100);

        //move away from the backdrop
        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .forward(8)//
                .build();
        drive.followTrajectory(trajectory5);
        //sleep(10);

        //move the arm and slide down
        slide.moveToWhiteStripWithoutWaiting(0, 1); //low
        //sleep(100);

        if (Math.abs(strafeRightInches) > 1)
        {
            Trajectory tra = drive.trajectoryBuilder(trajectory5.end())
                    .strafeRight(strafeRightInches)
                    .build();
             drive.followTrajectory(tra);
        }
    }

    /**
     * plus 2 pixels
     * @param drive
     * @param slide
     * @param outtake
     * @param arm
     */
    private void plusLeftRoute(SampleMecanumDrive drive,
                               PicassoSlide slide,
                               Intake intake,
                               Outtake outtake,
                               Arm arm)
    {
        //move the starting point
        Trajectory trajectory0 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(1,0, Math.toRadians(-94)))
                .build();
        drive.followTrajectory(trajectory0);
        //sleep(10);

        //adjust heading here
        //adjust heading
        double deltaAngle = drive.adjustHeading(-90);
        if(Math.abs(deltaAngle) > 0.3)
            sleep(50);


        //move forward to the blue right starting position
        Trajectory trajectory1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(48) //28
                .build();
        drive.followTrajectory(trajectory1);
        //sleep(50);

        //arm down
        arm.goDown();

        //go to in the front of the right stack
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(27,-66, Math.toRadians(-94)))//26.5,-66
                .build();
        drive.followTrajectory(trajectory2);

        //intake the pixels
        plusIntake(drive, intake, outtake);

        //move back to the blue right starting position
        Trajectory trajectory3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(0,-50, Math.toRadians(-94)))
                .build();
        drive.followTrajectory(trajectory3);

        //adjust heading
        deltaAngle = drive.adjustHeading(-90);
        if(Math.abs(deltaAngle) > 0.3)
            sleep(50);

        //move to backstage
        Trajectory trajectory4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(60)
                .build();
        drive.followTrajectory(trajectory4);
        //sleep(50);
    }

    /**
     * plus 2 pixels
     * @param drive
     * @param slide
     * @param outtake
     * @param arm
     */
    private void plusCenterRoute(SampleMecanumDrive drive,
                              PicassoSlide slide,
                              Intake intake,
                              Outtake outtake,
                              Arm arm)
    {
        Trajectory trajectory0 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(27.5,0, Math.toRadians(-94)))
                .build();
        drive.followTrajectory(trajectory0);
        //sleep(10);

        //adjust heading here
        double deltaAngle = drive.adjustHeading(-90);
        if(Math.abs(deltaAngle) > 0.3)
            sleep(50);


        //move the position
        Trajectory trajectory1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(48) //28
                .build();
        drive.followTrajectory(trajectory1);

        //arm down
        arm.goDown();

        //go to in the front of the right stack
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(26,-66, Math.toRadians(-94)))//27,-66
                .build();
        drive.followTrajectory(trajectory2);

        //intake the pixels
        plusIntake(drive, intake, outtake);

        //go back
        Trajectory trajectory3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(26,-50, Math.toRadians(-94)))
                .build();
        drive.followTrajectory(trajectory3);

        //adjust heading
        deltaAngle = drive.adjustHeading(-90);
        if(Math.abs(deltaAngle) > 0.3)
            sleep(50);

        //move to backstage
        Trajectory trajectory4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(60)
                .build();
        drive.followTrajectory(trajectory4);
    }

    public void plusIntake(SampleMecanumDrive drive,
                           Intake intake,
                           Outtake outtake)
    {
        //disturb pixels
        intake.setPower(-0.3);

        Trajectory trajectory_i1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(7)
                .build();
        drive.followTrajectory(trajectory_i1);
        sleep(50);

        //take in pixels
        outtake.TakeIn();
        intake.setPower(0.7);

        Trajectory trajectory_i2 = drive.trajectoryBuilder(trajectory_i1.end())
                .back(7)
                .build();
        drive.followTrajectory(trajectory_i2);

        Trajectory trajectory_i3 = drive.trajectoryBuilder(trajectory_i2.end())
                .forward(7)
                .build();
        drive.followTrajectory(trajectory_i3);
        sleep(50);//100

        Trajectory trajectory_ie = drive.trajectoryBuilder(trajectory_i3.end())
                .back(7)
                .build();
        drive.followTrajectory(trajectory_ie);

        //spit out extra pixels
        outtake.Hold();
        intake.setPower(-0.85);
        sleep(300);//350
        intake.setPower(0);
    }


}

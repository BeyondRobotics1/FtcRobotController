package org.firstinspires.ftc.teamcode.centerstage.picasso;

//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Right Right", group="Picasso")
@Disabled

public class CCAutoRedRight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
    }
//    OpenCvWebcam webcam;
//    TeamPropDeterminationPipeline pipeline;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        //
//        telemetry.addLine("Initializing drive train");
//        telemetry.update();
//
//        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        //
//        telemetry.addLine("Initializing camera");
//        telemetry.update();
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        // OR...  Do Not Activate the Camera Monitor View
//        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
//
//        /*
//         * Specify the image processing pipeline we wish to invoke upon receipt
//         * of a frame from the camera. Note that switching pipelines on-the-fly
//         * (while a streaming session is in flight) *IS* supported.
//         */
//        pipeline = new TeamPropDeterminationPipeline(TeamPropDeterminationPipeline.TeamPropColor.RED, this);
//        webcam.setPipeline(pipeline);
//
//        /*
//         * Open the connection to the camera device. New in v1.4.0 is the ability
//         * to open the camera asynchronously, and this is now the recommended way
//         * to do it. The benefits of opening async include faster init time, and
//         * better behavior when pressing stop during init (i.e. less of a chance
//         * of tripping the stuck watchdog)
//         *
//         * If you really want to open synchronously, the old method is still available.
//         */
//        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                /*
//                 * Tell the webcam to start streaming images to us! Note that you must make sure
//                 * the resolution you specify is supported by the camera. If it is not, an exception
//                 * will be thrown.
//                 *
//                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
//                 * supports streaming from the webcam in the uncompressed YUV image format. This means
//                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
//                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
//                 *
//                 * Also, we specify the rotation that the webcam is used in. This is so that the image
//                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
//                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
//                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
//                 * away from the user.
//                 */
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
//
//
//        TeamPropDeterminationPipeline.TeamPropPosition position = TeamPropDeterminationPipeline.TeamPropPosition.CENTER;
//
//        double satRectLeft = 0;
//        double satRectMiddle = 0;
//        double satRectRight = 0;
//
//        Boolean plus2 = true;
//
//        while (!isStarted() && !isStopRequested()) {
//            if(gamepad1.y)
//                plus2 = true;
//            else if(gamepad1.x)
//                plus2 = false;
//
//            position = pipeline.getAnalysis();
//            satRectLeft = pipeline.getSatRectLeft();
//            satRectMiddle = pipeline.getSatRectMiddle();
//            satRectRight = pipeline.getSatRectRight();
//
//            telemetry.addData("Auto Red Right. Plus 2", "%s", plus2?" Enabled":"Disabled");
//            telemetry.addLine("X - Disable +2, Y - Enable +2");
//            telemetry.addData("L", "%.2f", satRectLeft);
//            telemetry.addData("C", "%.2f", satRectMiddle);
//            telemetry.addData("R", "%.2f", satRectRight);
//
//            telemetry.addData("Realtime analysis", position);
//            telemetry.update();
//
//            //telemetry.addData("Auto Blue Left: Position Detected", position);
//            //telemetry.update();
//
//            // Don't burn CPU cycles busy-looping in this sample
//            sleep(100);
//        }
//
//        //locking the pixel
//        PixelPlacer pixelPlacer = new PixelPlacer(hardwareMap, this);
//
//
//        if (isStopRequested()) return;
//
//        PicassoSlide slide = new PicassoSlide(hardwareMap, this);
//        slide.runWithEncoder();
//        Intake intake = new Intake(hardwareMap, this);
//
//        Outtake outtake = new Outtake(hardwareMap, this);
//        Arm arm = new Arm(hardwareMap, this);
//
//        switch (position)
//        {
//            case LEFT:
//            {
//                //move the left mark position
//                Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
//                        .forward(28)
//                        .build();
//                drive.followTrajectory(trajectory1);
//
//                drive.turn(Math.toRadians(-94));
//
//                Trajectory trajectory2 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                        .back(6)//7
//                        .build();
//                drive.followTrajectory(trajectory2);
//
//                //unlock the pixel
//                pixelPlacer.Unlock();
//                sleep(50);
//
//                //move forward
//                Trajectory trajectory22 = drive.trajectoryBuilder(trajectory2.end())
//                        .forward(10)
//                        .build();
//                drive.followTrajectory(trajectory22);
//
//                double strafeLeftInches = 30;//
//
//                placePixelAndPark(drive, slide, outtake, arm, 7.9,36, -38, 90, strafeLeftInches);//36.5
//
//                break;
//            }
//
//            case CENTER:
//            {
//                //move the center mark position
//                Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
//                        .forward(42)//43
//                        .build();
//                drive.followTrajectory(trajectory1);
//
//                //unlock the pixel
//                pixelPlacer.Unlock();
//                sleep(50);
//
//                //move forward
//                Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
//                        .forward(7.5)//13
//                        .build();
//                drive.followTrajectory(trajectory2);
//                //sleep(100);
//
//                Trajectory tra = drive.trajectoryBuilder(trajectory2.end())
//                        .strafeRight(15)
//                        .build();
//                drive.followTrajectory(tra);
//                //sleep(100);
//
//                double strafeLeftInches = 25;//
//                if(plus2)
//                    strafeLeftInches = 0;
//
//                placePixelAndPark(drive, slide, outtake, arm, 7.9,24.5, -37, 90, strafeLeftInches);//18.5, 32, -90
//
//                //test
//                if(plus2)
//                {
//                    plusRightRoute(drive, slide, intake, outtake, arm,
//                            new Pose2d(3,0, Math.toRadians(89)),//3, -2
//                            new Pose2d(32.5,70, Math.toRadians(89)),
//                            new Pose2d(3.5,46, Math.toRadians(89)),
//                            90, 90);//58
//
//                    //center backdrop
//                    placePixelAndPark(drive, slide, outtake, arm, 14.5, 35, -37, 90, 20); //24.5, 32, -90
//                }
//
//                break;
//            }
//
//            case RIGHT:
//            {
//                //move to the right mark position in a straight line
//                Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
//                        .lineToLinearHeading(new Pose2d(32,-14))//35,-13.5
//                        .build();
//                drive.followTrajectory(trajectory1);
//
//                //unlock the pixel
//                pixelPlacer.Unlock();
//                sleep(50);//200
//
//                //move forward
//                Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
//                        .forward(15.5)//12
//                        .build();
//                drive.followTrajectory(trajectory2);
//
//                double strafeLeftInches = 20;//
//                if(plus2)
//                    strafeLeftInches = 0;
//
//                placePixelAndPark(drive, slide, outtake, arm, 7.9,20, -37, 90, strafeLeftInches);//18.5, 32, -90
//
//                //test
//                if(plus2)
//                {
//                    plusRightRoute(drive, slide, intake, outtake, arm,
//                            new Pose2d(3,0, Math.toRadians(92)),//3, -2, 88
//                            new Pose2d(32.5,70, Math.toRadians(92)),//30, 70
//                            new Pose2d(2,46, Math.toRadians(92)),
//                            90, 90);//
//
//                    //center backdrop
//                   placePixelAndPark(drive, slide, outtake, arm, 14.5, 35, -37, 90, 20); //24.5, 32, -90
//                }
//
//                break;
//            }
//
//
//        }
//
//        sleep(5000);
//    }
//
//    private void placePixelAndPark(SampleMecanumDrive drive,
//                                   PicassoSlide slide,
//                                   Outtake outtake,
//                                   Arm arm,
//                                   double slideHeight,
//                                   double x,
//                                   double y,
//                                   double heading,
//                                   double strafeLeftInches
//
//    )
//    {
//        //move slide up to one pixels high
//        arm.goUp();
//        slide.moveToWithoutWaiting(slideHeight, 1); //7.6
//
//        Trajectory trajectory4 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(heading)))
//                .build();
//        drive.followTrajectory(trajectory4);
//        sleep(50);
//
//        //outtake the pixel
//        outtake.TakeOut(0.65);
//        sleep(750);//500
//        outtake.Hold();
//
//        if (Math.abs(strafeLeftInches) > 1)
//        {
//            sleep(200);
//
//            //move away from the backdrop
//            Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
//                    .forward(5)//8
//                    .build();
//            drive.followTrajectory(trajectory5);
//
//            //move the arm and slide down
//            //arm.goDown();
//            slide.moveToWhiteStripWithoutWaiting(0, 1); //low
//
//            Trajectory tra = drive.trajectoryBuilder(trajectory5.end())
//                    .strafeLeft(strafeLeftInches)
//                    .build();
//            drive.followTrajectory(tra);
//        }
//    }
//    /**
//     * plus 2 pixels
//     * @param drive
//     * @param slide
//     * @param outtake
//     * @param arm
//     */
//    private void plusRightRoute(SampleMecanumDrive drive,
//                               PicassoSlide slide,
//                               Intake intake,
//                               Outtake outtake,
//                               Arm arm,
//                                Pose2d pos1,
//                                Pose2d pos2,
//                                Pose2d pos3,
//                                double angle1,
//                                double angle2)
//    {
//        //move the starting point
//        Trajectory trajectory0 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .lineToLinearHeading(pos1) //changed
//                .build();
//        drive.followTrajectory(trajectory0);
//
//        slide.moveToWhiteStripWithoutWaiting(0, 1); //low
//
//        //adjust heading
//        double deltaAngle = drive.adjustHeading(angle1);//94
//        if(Math.abs(deltaAngle) > 0.3)
//            sleep(50);
//
//        //move forward to the blue right starting position
//        Trajectory trajectory1 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .forward(56) //48
//                .build();
//        drive.followTrajectory(trajectory1);
//
//        //arm down
//        arm.goDown();
//
//
//        //disturb pixels
//        intake.setPower(-0.3);
//
//        //go to in the front of the right stack
//        //pose2
//        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
//                .lineToLinearHeading(pos2)//26.5,-66
//                .build();
//        drive.followTrajectory(trajectory2);
//
//
//        //take in pixels
//        outtake.TakeIn();
//        intake.setPower(0.7);
//        Trajectory trajectory_i2 = drive.trajectoryBuilder(trajectory2.end())
//                .back(4)
//                .build();
//        drive.followTrajectory(trajectory_i2);
//
//        intake.setPower(-0.5);
//        sleep(100);//100
//        intake.setPower(0.7);
//
//        Trajectory trajectory_i3 = drive.trajectoryBuilder(trajectory_i2.end())
//                .forward(6)
//                .build();
//        drive.followTrajectory(trajectory_i3);
//        sleep(50);//100
//
//        Trajectory trajectory_ie = drive.trajectoryBuilder(trajectory_i3.end())
//                .back(7)
//                .build();
//        drive.followTrajectory(trajectory_ie);
//
//        //spit out extra pixels
//        outtake.Hold();
//        intake.setPower(-0.85);
//        sleep(250);//300
//        intake.setPower(-0.5);
//
//
//        //move back to the blue right starting position
//        Trajectory trajectory3 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .lineToLinearHeading(pos3)//changed
//                .build();
//        drive.followTrajectory(trajectory3);
//
//        intake.setPower(0);
//
//        //adjust heading
//        deltaAngle = drive.adjustHeading(angle2);//92
//        if(Math.abs(deltaAngle) > 0.3)
//            sleep(50);
//
//        //move to backstage
//        Trajectory trajectory4 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .back(48)//53
//                .build();
//        drive.followTrajectory(trajectory4);
//    }

}

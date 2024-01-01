package org.firstinspires.ftc.teamcode.centerstage.picasso;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropDeterminationPipeline extends OpenCvPipeline {

    public TeamPropDeterminationPipeline(TeamPropColor teamPropColor, LinearOpMode mode){
        this.mode = mode;
        setHueRange(teamPropColor);

        calibration_frame_count = 0; //starting from 1
        region1_calibration = 0;
        region2_calibration = 0;
        region3_calibration = 0;
    }
    /*
     * An enum to define the team prop position
     */
    public enum TeamPropPosition
    {
        LEFT,
        RIGHT,
        CENTER
    }

    public enum TeamPropColor
    {
        RED,
        BLUE
    }

    LinearOpMode mode;

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile TeamPropPosition position = TeamPropPosition.CENTER;

    //hue range for Team Prop Color
    private float hueRangeLow; //
    private float hueRangeHigh;

    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,80);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(106,80);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(212,80);
    static final int REGION_WIDTH = 106;
    static final int REGION_HEIGHT = 80;

    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */

    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    //public Scalar lower = new Scalar(0, 0, 0); // HSV threshold bounds
    //public Scalar upper = new Scalar(255, 255, 255);

//    private Mat hsvMat = new Mat(); // converted image
//    private Mat hueMat = new Mat();
//    private Mat region1_hue = new Mat();
//    private Mat region2_hue = new Mat();
//    private Mat region3_hue = new Mat();

    private long region1_calibration;
    private long region2_calibration;
    private long region3_calibration;
    private int CALIBRATION_FRAMES = 90;
    private int calibration_frame_count;

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public TeamPropPosition getAnalysis()
    {
        return position;
    }

    @Override
    public void init(Mat firstFrame) {
        //processFrame(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input)
    {

        Mat hsvMat = new Mat(); // converted image
        Mat hueMat = new Mat();
        //Mat region1_hue = new Mat();
        //Mat region2_hue = new Mat();
        //Mat region3_hue = new Mat();

        //convert image color fom RGB to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

//        mode.telemetry.addData("hsvMat rows", hsvMat.rows());
//        mode.telemetry.addData("hsvMat cols", hsvMat.cols());


        ////extract the hue channel col
        // hue is column 0
        //saturation is column 1
        //value is column 2
        Core.extractChannel(hsvMat, hueMat, 0);

//        mode.telemetry.addData("hueMat rows", hueMat.rows());
//        mode.telemetry.addData("hueMat cols", hueMat.cols());


        //get the hue matrices for three regions
        Mat region1_hue = hueMat.submat(new Rect(region1_pointA, region1_pointB));
        Mat region2_hue = hsvMat.submat(new Rect(region2_pointA, region2_pointB));
        Mat region3_hue = hsvMat.submat(new Rect(region3_pointA, region3_pointB));

        if(calibration_frame_count < CALIBRATION_FRAMES)
        {
            region1_calibration += countInRange(region1_hue);
            region2_calibration += countInRange(region2_hue);
            region3_calibration += countInRange(region3_hue);


            calibration_frame_count++;

            mode.telemetry.addData("Calibrating frame", calibration_frame_count);
            mode.telemetry.addData("Calibrating Region 1", region1_calibration/calibration_frame_count);
            mode.telemetry.addData("Calibrating Region 2", region2_calibration/calibration_frame_count);
            mode.telemetry.addData("Calibrating Region 3", region3_calibration/calibration_frame_count);



            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    new Scalar(255, 0, 0), // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    new Scalar(0, 255, 0), // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    new Scalar(0, 0, 255), // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

        }
        else if (calibration_frame_count == CALIBRATION_FRAMES)
        {
            double r1 = region1_calibration/calibration_frame_count;
            double r2 = region2_calibration/calibration_frame_count;
            double r3 = region3_calibration/calibration_frame_count;

            region1_calibration = Math.round(r1);
            region2_calibration = Math.round(r2);
            region3_calibration = Math.round(r3);

            mode.telemetry.addData("Calibrated frames", calibration_frame_count);
            mode.telemetry.addData("Calibrated Region 1", region1_calibration);
            mode.telemetry.addData("Calibrated Region 2", region2_calibration);
            mode.telemetry.addData("Calibrated Region 3", region3_calibration);

            calibration_frame_count++;

        }
        else {

            mode.telemetry.addLine()
                    .addData("Region 1", region1_hue.rows())
                    .addData("x ", region1_hue.cols());
            mode.telemetry.addLine()
                    .addData("Region 2", region2_hue.rows())
                    .addData("x ", region2_hue.cols());
            mode.telemetry.addLine()
                    .addData("Region 3", region3_hue.rows())
                    .addData("x ", region3_hue.cols());


            long region1_count = countInRange(region1_hue);
            long region2_count = countInRange(region2_hue);
            long region3_count = countInRange(region3_hue);

            mode.telemetry.addLine()
                    .addData("Region 1 base", region1_calibration)
                    .addData("raw", region1_count);
            mode.telemetry.addLine()
                    .addData("Region 2 base", region2_calibration)
                    .addData("raw", region2_count);
            mode.telemetry.addLine()
                    .addData("Region 3 base", region3_calibration)
                    .addData("raw", region3_count);

            region1_count -= region1_calibration;
            region2_count -= region2_calibration;
            region3_count -= region3_calibration;


            if (region1_count > region2_count && region1_count > region3_count) {
                position = TeamPropPosition.LEFT;
            } else if (region3_count > region1_count && region3_count > region2_count) {
                position = TeamPropPosition.RIGHT;
            } else
                position = TeamPropPosition.CENTER;

            mode.telemetry.addData("Calibrated Region 1", region1_count);
            mode.telemetry.addData("Calibrated Region 2", region2_count);
            mode.telemetry.addData("Calibrated Region 3", region3_count);
            mode.telemetry.addData("Realtime analysis", position);
            mode.telemetry.update();


            if (position == TeamPropPosition.LEFT)
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        new Scalar(255, 0, 0), // The color the rectangle is drawn in
                        6); // Thickness of the rectangle lines
            else
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        new Scalar(255, 0, 0), // The color the rectangle is drawn in
                        2); // Thickness of the rectangle lines


            if (position == TeamPropPosition.CENTER)
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        new Scalar(0, 255, 0), // The color the rectangle is drawn in
                        6); // Thickness of the rectangle lines
            else
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        new Scalar(0, 255, 0), // The color the rectangle is drawn in
                        2); // Thickness of the rectangle lines


            if (position == TeamPropPosition.RIGHT)
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        new Scalar(0, 0, 255), // The color the rectangle is drawn in
                        6); // Thickness of the rectangle lines
            else
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        new Scalar(0, 0, 255), // The color the rectangle is drawn in
                        2); // Thickness of the rectangle lines

        }

        return input;
    }

    private long countInRange(Mat hueMat)
    {
        long count = 0;
        double hue;

        for (int i = 0; i < hueMat.rows(); i++) {
            for (int j = 0; j < hueMat.cols(); j++) {
                hue = hueMat.get(i, j)[0];

                //add 180 when less than 20 so we can use just one range
                if(hue < 20)
                    hue += 180;

                if (hue >= hueRangeLow && hue <= hueRangeHigh) {
                    count++;
                }
            }
        }

        return count;
    }

    //set the hue range for color Red or Blue
    private void setHueRange(TeamPropColor teamPropColor)
    {
        //for red 0 - 20 and 160 - 180
        //add 180 when less than 20 so we can use just one range
        if(teamPropColor == TeamPropColor.RED)
        {
            hueRangeLow = 160;
            hueRangeHigh = 200;
        }
        else //blue
        {
            hueRangeLow = 100;//100
            hueRangeHigh = 130;//140
        }
    }
}

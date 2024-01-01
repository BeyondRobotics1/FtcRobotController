package org.firstinspires.ftc.teamcode.centerstage.picasso;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class TeamPropDeterminationPipeline extends OpenCvPipeline {

    public TeamPropDeterminationPipeline(TeamPropColor teamPropColor, LinearOpMode mode){
        this.mode = mode;
        setHueRange(teamPropColor);

        region1_history = new long[CALIBRATION_FRAMES];
        region2_history = new long[CALIBRATION_FRAMES];
        region3_history = new long[CALIBRATION_FRAMES];

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


    private Mat hsvMat = new Mat(); // converted image
    private Mat hueMat = new Mat();
    private Mat region1_hue = new Mat();
    private Mat region2_hue = new Mat();
    private Mat region3_hue = new Mat();

    private long region1_calibration;
    private long region2_calibration;
    private long region3_calibration;
    long region1_count;
    long region2_count;
    long region3_count;


    long[] region1_history;
    long[] region2_history;
    long[] region3_history;

    private int ignored_frames_count = 0;

    private int CALIBRATION_FRAMES = 240;
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
        testWithSaturation(input);

        return input;
    }

    //@Override
    private void testWithHue(Mat input)
    {
        ignored_frames_count++;

        if(ignored_frames_count < 10)
            return ;

        //convert image color fom RGB to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        ////extract the hue channel col
        // hue is column 0
        //saturation is column 1
        //value is column 2
        Core.extractChannel(hsvMat, hueMat, 0);


        //get the hue matrices for three regions
        region1_hue = hueMat.submat(new Rect(region1_pointA, region1_pointB));
        region2_hue = hsvMat.submat(new Rect(region2_pointA, region2_pointB));
        region3_hue = hsvMat.submat(new Rect(region3_pointA, region3_pointB));


        if(calibration_frame_count < CALIBRATION_FRAMES)
        {

            region1_history[calibration_frame_count] = countInRange(region1_hue);
            region2_history[calibration_frame_count] = countInRange(region2_hue);
            region3_history[calibration_frame_count] = countInRange(region3_hue);

            mode.telemetry.addLine()
                    .addData("Calibrating", calibration_frame_count)
                    .addData("L", region1_history[calibration_frame_count])
                    .addData("C", region2_history[calibration_frame_count])
                    .addData("R", region3_history[calibration_frame_count]);

            calibration_frame_count++;

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
            for(int i = 0; i < CALIBRATION_FRAMES; i++) {
                region1_calibration += region1_history[i];
                region2_calibration += region2_history[i];
                region3_calibration += region3_history[i];
            }

            region1_calibration = Math.round(region1_calibration * 1.0 / calibration_frame_count);
            region2_calibration = Math.round(region2_calibration * 1.0 / calibration_frame_count);
            region3_calibration = Math.round(region3_calibration * 1.0 / calibration_frame_count);

            calibration_frame_count++;

        }
        else {

            region1_count = countInRange(region1_hue);
            region2_count = countInRange(region2_hue);
            region3_count = countInRange(region3_hue);

            mode.telemetry.addLine()
                    .addData("TH", hueRangeHigh)
                    .addData("TL", hueRangeLow);

            mode.telemetry.addData("Calibrated frames", calibration_frame_count - 1);
            mode.telemetry.addLine()
                    .addData("L base", region1_calibration)
                    .addData("raw", region1_count);
            mode.telemetry.addLine()
                    .addData("C base", region2_calibration)
                    .addData("raw", region2_count);
            mode.telemetry.addLine()
                    .addData("R base", region3_calibration)
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

            mode.telemetry.addLine()
                    .addData("Calibrated L", region1_count)
                    .addData("C", region2_count)
                    .addData("R", region3_count);

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
            hueRangeHigh = 125;//140
        }
    }

    private void testWithSaturation(Mat input)
    {
        //convert image color fom RGB to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);


        //get the hue matrices for three regions
        region1_hue = hsvMat.submat(new Rect(region1_pointA, region1_pointB));
        region2_hue = hsvMat.submat(new Rect(region2_pointA, region2_pointB));
        region3_hue = hsvMat.submat(new Rect(region3_pointA, region3_pointB));


        double satRectLeft = Core.mean(region1_hue).val[1];
        double satRectMiddle = Core.mean(region2_hue).val[1];
        double satRectRight = Core.mean(region3_hue).val[1];

        mode.telemetry.addLine()
                .addData("L", "%.3f", satRectLeft)
                .addData("C", "%.3f", satRectMiddle)
                .addData("R", "%.3f", satRectRight);

        mode.telemetry.addData("Realtime analysis", position);
        mode.telemetry.update();

        if ((satRectLeft > satRectMiddle) && (satRectLeft > satRectRight)) {
            position = TeamPropPosition.LEFT;
        } else if ((satRectMiddle > satRectLeft) && (satRectMiddle > satRectRight)) {
            position = TeamPropPosition.CENTER;
        }
        else
            position = TeamPropPosition.RIGHT;

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


}

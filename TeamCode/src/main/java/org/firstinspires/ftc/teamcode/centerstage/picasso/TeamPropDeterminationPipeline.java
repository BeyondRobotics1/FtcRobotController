package org.firstinspires.ftc.teamcode.centerstage.picasso;

import org.firstinspires.ftc.teamcode.easyopencv.SkystoneDeterminationExample;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropDeterminationPipeline extends OpenCvPipeline {

    public TeamPropDeterminationPipeline(TeamPropColor teamPropColor){}
    /*
     * An enum to define the team prop position
     */
    public enum TeamPropPosition
    {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum TeamPropColor
    {
        RED,
        BLUE
    }

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

    private Mat hsvMat = new Mat(); // converted image
    private Mat binaryMat = new Mat(); // image analyzed after thresholding
    private Mat hueMat = new Mat();

    Mat region1_hue, region2_hue, region3_hue;

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public TeamPropPosition getAnalysis()
    {
        return position;
    }

    @Override
    public void init(Mat firstFrame) {

    }

    @Override
    public Mat processFrame(Mat input)
    {
        //convert image color fom RGB to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        //extract the hue channel
        Core.extractChannel(hsvMat, hueMat, 0);

        //get the hue matrices for three regions
        region1_hue = hueMat.submat(new Rect(region1_pointA, region1_pointB));
        region2_hue = hueMat.submat(new Rect(region2_pointA, region2_pointB));
        region3_hue = hueMat.submat(new Rect(region3_pointA, region3_pointB));

        int region1_count = countInRange(region1_hue);
        int region2_count = countInRange(region2_hue);
        int region3_count = countInRange(region3_hue);

        if(region1_count >= region2_count &&  region1_count >= region3_count)
        {
            position = TeamPropPosition.LEFT;
        }
        else if(region3_count >= region1_count &&  region3_count >= region2_count)
        {
            position = TeamPropPosition.RIGHT;
        }
        else
            position = TeamPropPosition.CENTER;

        if(position == TeamPropPosition.LEFT)
            Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                new Scalar(0, 255, 0), // The color the rectangle is drawn in
                4); // Thickness of the rectangle lines
        else
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    new Scalar(0, 255, 0), // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


        if(position == TeamPropPosition.CENTER)
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    new Scalar(0, 255, 0), // The color the rectangle is drawn in
                    4); // Thickness of the rectangle lines
        else
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    new Scalar(0, 255, 0), // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


        if(position == TeamPropPosition.RIGHT)
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    new Scalar(0, 255, 0), // The color the rectangle is drawn in
                    4); // Thickness of the rectangle lines
        else
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    new Scalar(0, 255, 0), // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

        //
        return binaryMat;
    }

    private int countInRange(Mat hueMat)
    {
        int count = 0;

        for (int i = 0; i <= hueMat.cols(); i++) {
            for (int j = 0; j <= hueMat.rows(); j++) {
                double hue = hueMat.get(i, j)[0];

                //add 180 when less than 20 so we can use just one range
                if(hue < 20)
                    hue += 180;

                if (hue >= hueRangeLow && hue <= hueRangeHigh) {
                    count++;
                }
            }
        }

        return 0;
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
        else
        {
            hueRangeLow = 100;
            hueRangeHigh = 140;
        }
    }
}

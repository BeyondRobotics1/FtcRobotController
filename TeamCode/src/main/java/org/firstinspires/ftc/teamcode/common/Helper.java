package org.firstinspires.ftc.teamcode.common;

public class Helper {

    public static final double TAU = Math.PI * 2;

    public static double cubicWithSign(double input)
    {
        double output = input * input * input;

        return output;
    }

    public static double squareWithSign(double input)
    {
        double output = input * input;

        if(input < 0)
            return -output;
        else
            return output;
    }


    /**
     * Returns [angle] clamped to `[0, 2pi]`.
     *
     * @param angle angle measure in radians
     */
    public static double norm(double angle) {
        double modifiedAngle = angle % Helper.TAU;

        modifiedAngle = (modifiedAngle + Helper.TAU) % Helper.TAU;

        return modifiedAngle;
    }

    /**
     * Returns [angleDelta] clamped to `[-pi, pi]`.
     *
     * @param angleDelta angle delta in radians
     */
    public static double normDelta(double angleDelta) {
        double modifiedAngleDelta = norm(angleDelta);

        if (modifiedAngleDelta > Math.PI) {
            modifiedAngleDelta -= Helper.TAU;
        }

        return modifiedAngleDelta;
    }

    /**
     * scale a value from [oldMin oldMax] to [newMin newMax]
     * @param value
     * @param oldMin
     * @param oldMax
     * @param newMin
     * @param newMax
     * @return
     */
    public static double Scale(double value, double oldMin, double oldMax, double newMin, double newMax) {
        return (value - oldMin) * (newMax - newMin) / (oldMax - oldMin) + newMin;
    }
}

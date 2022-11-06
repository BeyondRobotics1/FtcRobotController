package org.firstinspires.ftc.teamcode;

public class Helper {

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
}

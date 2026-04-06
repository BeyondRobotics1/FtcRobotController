package org.firstinspires.ftc.teamcode.decode.Subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {

    public enum IntakeMode
    {
        IN,
        VIN,
        HIN,
        OUT,
        SLOW_FEED,
        MEDIUM_FEED,
        FEED,
        IDLE
    }

    public static double LED_OFF = 0;
    public static double LED_RED = 0.277;
    public static double LED_ORANGE = 0.333;
    public static double LED_YELLOW = 0.388;
    public static double LED_SAGE = 0.444;
    public static double LED_GREEN = 0.5;
    public static double LED_Azure = 0.555;
    public static double LED_BLUE = 0.611;
    public static double LED_INDIGO = 0.666;
    public static double LED_VIOLET = 0.722;

    private DcMotor intakeMotor;
    private DcMotor transferMotor;
    private Servo led;
    private LinearOpMode mode;


    NormalizedColorSensor colorSensorTop;
    NormalizedColorSensor colorSensorMiddle;
    NormalizedColorSensor colorSensorFrontLeft;
    NormalizedColorSensor colorSensorFrontRight;
    float[] hsvValues;
    int[] artifactColors; //0 - top, 1 - middle, 2 - bottom
    int[] latchCounters; //0 - top, 1 - middle, 2 - bottom

    int latchLimit = 3;


    public Intake(HardwareMap hardwareMap, LinearOpMode linearOpMode)
    {
        this.mode = linearOpMode;
        intakeMotor = hardwareMap.get(DcMotor.class, "horizontalIntake");
        transferMotor = hardwareMap.get(DcMotor.class, "verticalIntake");

        led = hardwareMap.get(Servo.class, "led");

        //verticalIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorSensorTop = hardwareMap.get(NormalizedColorSensor.class, "colorTop");
        colorSensorMiddle = hardwareMap.get(NormalizedColorSensor.class, "colorMiddle");
        colorSensorFrontLeft = hardwareMap.get(NormalizedColorSensor.class, "colorFrontLeft");
        colorSensorFrontRight = hardwareMap.get(NormalizedColorSensor.class, "colorFrontRight");

        colorSensorTop.setGain(2.0f);
        colorSensorMiddle.setGain(2.0f);
        colorSensorFrontLeft.setGain(2.0f);
        colorSensorFrontRight.setGain(2.0f);

        hsvValues = new float[3];

        artifactColors = new int[3];
        latchCounters = new int[3];

        latchCounters[0] = latchCounters[1] = latchCounters[2] = latchLimit;
    }

    /**
     * Set the spinner to intake or outtake mode
     * @param intakeMode
     */
    public void setIntakeMode(IntakeMode intakeMode)
    {
        if(intakeMode == Intake.IntakeMode.IN)
        {
            intake(0.925);
        }
        else if(intakeMode == Intake.IntakeMode.VIN)
        {
            intakeMotor.setPower(0);
            transferMotor.setPower(-1);
        }
        if(intakeMode == Intake.IntakeMode.HIN)
        {
            intakeMotor.setPower(-0.9);
            transferMotor.setPower(0);
        }
        else if (intakeMode == Intake.IntakeMode.OUT) {

            //outtake(0.5);
            intakeMotor.setPower(0.5);
            transferMotor.setPower(0.5);
        }
        else if (intakeMode == IntakeMode.FEED) {
            intake(1);
            //horizontalIntake.setPower(1);
            //verticalIntake.setPower(1);
        }
        else if (intakeMode == IntakeMode.SLOW_FEED)
        {
            intake(0.65); //0.65
        }
        else if (intakeMode == IntakeMode.MEDIUM_FEED)
        {
            intake(0.8);
        }
        else {
            intakeMotor.setPower(0);
            transferMotor.setPower(0);
        }
    }

    //Power should be negative
    public void intake(double power)
    {
        double localPower = -Math.abs(power);

        intakeMotor.setPower(localPower);
        transferMotor.setPower(localPower);

//        mode.telemetry.addLine()
//                .addData("horizontal Power", "%.3f", -localPower)
//                .addData("vertical Power", "%.3f", -localPower);
    }

    public void intake(double horizontalIntakePower,
                       double verticalIntakePower)
    {
        intakeMotor.setPower( -Math.abs(horizontalIntakePower));
        transferMotor.setPower( -Math.abs(verticalIntakePower));
    }

    //Power should be positive
    public void outtake(double power)
    {
        double localPower = Math.abs(power);
        intakeMotor.setPower(localPower);
        transferMotor.setPower(localPower);
    }

    public void stopVertical()
    {
        transferMotor.setPower(0);
    }


    public int detectedArtifacts() {
        if (artifactColors[0] != Color.WHITE &&
                artifactColors[1] != Color.WHITE &&
                artifactColors[2] != Color.WHITE)
            return 3;
        else if (artifactColors[0] != Color.WHITE &&
                artifactColors[1] != Color.WHITE)
            return 2;
        else if (artifactColors[0] != Color.WHITE)
            return 1;
        else
            return 0;
    }

    //detect if there is an artifact at top, middle, and bottom
    //location of the transfer system
    //And assign the color to the array.
    //Color.WHITE means no artifact is detected
    public int[] detectArtifactColors()
    {
        int topColor = getTopArtifactColor();
        int middleColor = getMiddleArtifactColor();
        int bottomColor = getBottomArtifactColor();

        latch(0, topColor);
        latch(1, middleColor);
        latch(2, bottomColor);

        //artifactColors[0] = getTopArtifactColor();
        //artifactColors[1] = getMiddleArtifactColor();
        //artifactColors[2] = getBottomArtifactColor();

        return artifactColors;
    }


    public int getTopArtifactColor()
    {
        return detectArtifactColor(colorSensorTop);
    }

    public int getMiddleArtifactColor()
    {
        return detectArtifactColor(colorSensorMiddle);
    }

    public int getBottomArtifactColor()
    {
        int bottomLeft = detectArtifactColor(colorSensorFrontLeft);
        int bottomRight = detectArtifactColor(colorSensorFrontRight);

        if(bottomLeft != Color.WHITE)
            return bottomLeft;
        else if (bottomRight != Color.WHITE)
            return bottomRight;
        else
            return Color.WHITE;
    }


    //No PURPLE color predefined, use BLUE for PURPLE
    //WHITE means artifact is detected
    public int detectArtifactColor(NormalizedColorSensor colorSensor)
    {
        int color = Color.WHITE;

        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        Color.colorToHSV(colors.toColor(), hsvValues);

        if(((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < 4)
        { //
            if (hsvValues[0] >= 140 && hsvValues[0] <= 200)
                color = Color.GREEN;
            else if (hsvValues[0] > 200 && hsvValues[0] < 250)
                color = Color.BLUE;
        }

//        mode.telemetry.addLine()
//                .addData("Red", "%.3f", colors.red)
//                .addData("Green", "%.3f", colors.green)
//                .addData("Blue", "%.3f", colors.blue);
//        mode.telemetry.addLine()
//                .addData("Hue", "%.3f", hsvValues[0])
//                .addData("Saturation", "%.3f", hsvValues[1])
//                .addData("Value", "%.3f", hsvValues[2]);
//        mode.telemetry.addData("Alpha", "%.3f", colors.alpha);

        return color;
    }

    private void latch(int index, int color)
    {
        if(index < 0 || index > 2)
            return;

        if(color != Color.WHITE)
        {
            artifactColors[index] = color;
            latchCounters[index] = latchLimit;
        }
        else
        {
            //already reached the latch count down
            if(latchCounters[index] <= 0)
            {
                artifactColors[index] = color;
                latchCounters[index] = latchLimit;
            }
            else //not yet, count down
                latchCounters[index] -= 1;
        }
    }

    public void setLedColor(double colorPosition)
    {
        if(led != null) {
            led.setPosition(colorPosition);
        }
    }
}

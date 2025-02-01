package org.firstinspires.ftc.teamcode.intothedeep.Subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
        OUT,
        IDLE
    }

    public enum IntakePosition
    {
        HEAD_DOWN,
        INTAKE,
        OUTTAKE,
        NONE,
    }

    public enum SampleColor
    {
        BLUE,
        RED,
        YELLOW,
        NONE,
    }

    //pivot servo predefined positions, adjust as needed
    private final double pivotServoReadyPosition = 0.25; //
    private final double pivotServoHeadDownPosition = 0.4; //0
    private final double pivotServoIntakePosition = 0.68; //
    private final double pivotServoOuttakePosition = 0.84; //0.8


    //4bar servo predefined positions, adjust them as needed
    private final double fourBarServoIntakePosition = 0.11;//
    private final double fourBarServoReadyPosition = 0.3;//0.3
    private final double fourBarServoHeadDownPosition = 0.65;//0.4
    private final double fourBarServoOuttakePosition = 0.9;//0.85



    private IntakePosition currentPosition;

    private Servo fourBarServo;
    private Servo pivotServo;
    private Servo intakeServo;

    private Servo led;

    private NormalizedColorSensor colorSensor;

    private LinearOpMode mode;
    private final float[] hsvValues = new float[3];
    SampleColor lastSampleColor = SampleColor.NONE;
    /**
     * Constructor
     * @param hardwareMap: hardware map for finding claw servos
     * @param mode: for telemetry functions
     */
    public Intake(HardwareMap hardwareMap, LinearOpMode mode){

        this.mode = mode;

        fourBarServo = hardwareMap.get(Servo.class, "intakeFourBar");
        pivotServo = hardwareMap.get(Servo.class, "intakePivot");
        intakeServo = hardwareMap.get(Servo.class, "intakeSpinner");

        pivotServo.setDirection(Servo.Direction.REVERSE);
        intakeServo.setDirection(Servo.Direction.REVERSE);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color1");
        //led = hardwareMap.get(Servo.class, "led");

        currentPosition = IntakePosition.NONE;
    }



    public void MoveToIntakePosition()
    {
        //if(currentPosition != IntakePosition.INTAKE)
         //{

            fourBarServo.setPosition(fourBarServoIntakePosition);
            pivotServo.setPosition(pivotServoIntakePosition);

            currentPosition = IntakePosition.INTAKE;
        //}
    }


    public void MoveToReadyPosition()
    {
        //if(currentPosition != IntakePosition.HEAD_DOWN)
        //{
        fourBarServo.setPosition(fourBarServoReadyPosition);
        pivotServo.setPosition(pivotServoReadyPosition);

        currentPosition = IntakePosition.HEAD_DOWN;
        //}
    }

    public void MoveToOuttakePosition()
    {
        //if(currentPosition != IntakePosition.OUTTAKE)
        //{
            pivotServo.setPosition(pivotServoOuttakePosition);
            fourBarServo.setPosition(fourBarServoOuttakePosition);

            currentPosition = IntakePosition.OUTTAKE;
        //}
    }


    public void MoveToHeadDownPosition()
    {
        //if(currentPosition != IntakePosition.HEAD_DOWN)
        //{
        pivotServo.setPosition(pivotServoHeadDownPosition);
        fourBarServo.setPosition(fourBarServoHeadDownPosition);
        currentPosition = IntakePosition.HEAD_DOWN;
        //}
    }

    public void MoveToOuttakePositionAuto()
    {
        //if(currentPosition != IntakePosition.OUTTAKE)
        //{
        pivotServo.setPosition(pivotServoOuttakePosition);
        fourBarServo.setPosition(0.445);

        currentPosition = IntakePosition.OUTTAKE;
        //}
    }

    public double GetFourBarServoPosition()
    {
        return fourBarServo.getPosition();
    }

    public double GetPivotServoPosition()
    {
        return pivotServo.getPosition();
    }

    /**
     * Set the spinner to intake or outtake mode
     * @param intakeMode
     */
    public void SetIntakeSpinner(IntakeMode intakeMode)
    {
        if(intakeMode == IntakeMode.IN)
        {
            intakeServo.setPosition(1);
        }
        else if (intakeMode == IntakeMode.OUT) {
            intakeServo.setPosition(0);
        }
        else {
            intakeServo.setPosition(0.5);
        }
    }

    public void TestFourBarServo(double position)
    {
        fourBarServo.setPosition(position);
    }


    public void TestPivotServo(double position)
    {
        pivotServo.setPosition(position);
    }

    public void ResetSampleColor()
    {
        lastSampleColor = SampleColor.NONE;
    }

    public SampleColor GetSampleColor()
    {
        if(lastSampleColor == SampleColor.NONE)
            lastSampleColor = DetectSample();

        //if(lastSampleColor != SampleColor.NONE)
        return lastSampleColor;
    }

    public SampleColor DetectSample()
    {
        if(colorSensor == null)
            return SampleColor.NONE;

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        mode.telemetry.addData("distance", "%.3f", distance);
        mode.telemetry.addData("hue", "%.3f", hsvValues[0]);

        if( distance < 3.5) { //
            if (hsvValues[0] >= 50 && hsvValues[0] < 92) {

                //if(led != null)
                //    led.setPosition(0.388);

                mode.telemetry.addData("Color", "%s", "Yellow");
                return SampleColor.YELLOW;
            }
            else if ((hsvValues[0] >= 0 && hsvValues[0] <= 20) ||
                    (hsvValues[0] >= 340 && hsvValues[0] <= 360)) {
                //if(led != null)
                //    led.setPosition(0.279);
                mode.telemetry.addData("Color", "%s", "Red");
                return SampleColor.RED;
            }
            else if (hsvValues[0] >= 210 && hsvValues[0] <= 270) {
                //if(led != null)
                //    led.setPosition(0.611);
                mode.telemetry.addData("Color", "%s", "Blue");
                return SampleColor.BLUE;
            }
            //else if (hsvValues[0] >= 120 && hsvValues[0] <= 150)
            //    mode.telemetry.addData("Color", "%s", "Green");
            //else if (hsvValues[0] > 200 && hsvValues[0] < 230)
            //    mode.telemetry.addData("Color", "%s", "Purple");
            else {
                //if(led != null)
                //    led.setPosition(0);

                mode.telemetry.addData("Color", "%s", "None");

                return SampleColor.NONE;
            }
        }
        else {

            //if(led != null)
            //    led.setPosition(0);

            mode.telemetry.addData("Color", "%s", "None");
            return SampleColor.NONE;
        }

    }
}

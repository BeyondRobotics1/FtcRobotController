package org.firstinspires.ftc.teamcode.intothedeep.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

    //pivot servo predefined positions, adjust as needed
    private final double pivotServoReadyPosition = 0.25; //
    private final double pivotServoHeadDownPosition = 0.4; //0
    private final double pivotServoIntakePosition = 0.68; //
    private final double pivotServoOuttakePosition = 0.8; //


    //4bar servo predefined positions, adjust them as needed
    private final double fourBarServoIntakePosition = 0.11;//
    private final double fourBarServoReadyPosition = 0.3;//0.3
    private final double fourBarServoHeadDownPosition = 0.65;//0.4
    private final double fourBarServoOuttakePosition = 0.85;//



    private IntakePosition currentPosition;

    private Servo fourBarServo;
    private Servo pivotServo;
    private Servo intakeServo;

    private LinearOpMode mode;

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

}

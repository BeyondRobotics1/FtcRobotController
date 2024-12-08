package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    enum IntakeMode
    {
        IN,
        OUT,
        IDLE
    }

    enum IntakePosition
    {
        HEAD_DOWN,
        INTAKE,
        OUTTAKE,
        NONE,
    }

    //pivot servo predefined positions
    //private final double pivotServoStartPosition = 0.2;//leveled
    private final double pivotServoIntakePosition = 0.18;//0.85
    private final double pivotServoOuttakePosition = 0.1;//0.5
    private final double pivotServoHeadDownPosition = 0.5;//leveled

    //4bar servo predefined positions
    //private final double fourBarServoStartPosition = 0.6;//0.2;//
    private final double fourBarServoIntakePosition =0.92;//77
    private final double fourBarServoOuttakePosition = 0.485;
    private final double fourBarServoHeadDownPosition = 0.8;

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

        fourBarServo.setDirection(Servo.Direction.REVERSE);

        currentPosition = IntakePosition.NONE;

        //pivotServo.setPosition(pivotServoStartPosition);
        //fourBarServo.setPosition(fourBarServoStartPosition);

//        mode.telemetry.addLine().addData("intake4bar", "%.3f", fourBarServo.getPosition())
//                .addData("intakerotate", "%.3f", pivotServo.getPosition())
//                .addData("intakepivot", "%.3f", rotateServo.getPosition());
    }


    public void MoveToHeadDownPosition()
    {
        if(currentPosition != IntakePosition.HEAD_DOWN) {
            pivotServo.setPosition(pivotServoHeadDownPosition);
            fourBarServo.setPosition(fourBarServoHeadDownPosition);
            currentPosition = IntakePosition.HEAD_DOWN;
        }
    }

    public void MoveToIntakePosition()
    {
        if(currentPosition != IntakePosition.INTAKE) {

            pivotServo.setPosition(pivotServoIntakePosition);
            fourBarServo.setPosition(fourBarServoIntakePosition);

            currentPosition = IntakePosition.INTAKE;
        }
    }

    public void MoveToOuttakePosition()
    {
        if(currentPosition != IntakePosition.OUTTAKE)
        {
            pivotServo.setPosition(pivotServoOuttakePosition);
            fourBarServo.setPosition(fourBarServoOuttakePosition);

            currentPosition = IntakePosition.OUTTAKE;
        }
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

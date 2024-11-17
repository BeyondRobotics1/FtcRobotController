package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    enum IntakePosition
    {
        START,
        AIM,
        INTAKE,
        OUTTAKE,
        NONE,
    }

    boolean isRotationServoRotated;

    //rotation servo predefined positions
    final double rotationServoCenterPosition = 0.36;
    final double rotationServo90DegreePosition = 0.8;

    //pivot servo predefined positions
    final double pivotServoStartPosition = 0.9;//0.5
    final double pivotServoAimingPosition = 0.2;//0.2
    final double pivotServoIntakePosition = 0.2;//0.2
    final double pivotServoOuttakePosition = 0.68;//0.68

    //4bar servo predefined positions
    final double fourBarServoStartPosition = 0.3;//0.2;//
    final double fourBarServoAimingPosition =0.65;//
    final double fourBarServoIntakePosition =0.75;//77
    final double fourBarServoOuttakePosition = 0.58;

    IntakePosition currentPosition;

    Servo fourBarServo;
    Servo pivotServo;
    Servo rotateServo;

    LinearOpMode mode;

    /**
     * Constructor
     * @param hardwareMap: hardware map for finding claw servos
     * @param mode: for telemetry functions
     */
    public Intake(HardwareMap hardwareMap, LinearOpMode mode){

        this.mode = mode;

        fourBarServo = hardwareMap.get(Servo.class, "intake4bar");
        pivotServo = hardwareMap.get(Servo.class, "intakerotate");
        rotateServo = hardwareMap.get(Servo.class, "intakepivot");

        fourBarServo.setDirection(Servo.Direction.FORWARD);
        rotateServo.setDirection(Servo.Direction.FORWARD);
        pivotServo.setDirection(Servo.Direction.FORWARD);

        currentPosition = IntakePosition.NONE;

        //fourBarServo.setPosition(0.5);
        //pivotServo.setPosition(0.5);
        rotateServo.setPosition(rotationServoCenterPosition);
        isRotationServoRotated = false;
        //pivotServo.setPosition(pivotServoStartPosition);
        //fourBarServo.setPosition(fourBarServoStartPosition);

//        mode.telemetry.addLine().addData("intake4bar", "%.3f", fourBarServo.getPosition())
//                .addData("intakerotate", "%.3f", pivotServo.getPosition())
//                .addData("intakepivot", "%.3f", rotateServo.getPosition());
    }

    public void ChangeClawDirection(boolean to90Degree)
    {
        if(to90Degree) {
            if(!isRotationServoRotated) {
                rotateServo.setPosition(rotationServo90DegreePosition);
                isRotationServoRotated = true;
            }
        }
        else {
            if( isRotationServoRotated) {
                rotateServo.setPosition(rotationServoCenterPosition);
                isRotationServoRotated = false;
            }
        }
    }

    public void MoveToStartPosition()
    {
        if(currentPosition != IntakePosition.START) {

//            if(currentPosition == IntakePosition.AIM ||
//                    currentPosition == IntakePosition.INTAKE) {
//                pivotServo.setPosition(pivotServoStartPosition);
//                //mode.sleep(50);
//            }

            pivotServo.setPosition(pivotServoStartPosition);
            fourBarServo.setPosition(fourBarServoStartPosition);

            currentPosition = IntakePosition.START;
        }
    }

    public void MoveToAimingPosition()
    {

        if(currentPosition != IntakePosition.AIM) {

            //need to adjust the pivot if current position is START
            //otherwise, the pivot servo may block the four-bar servo
            if(currentPosition == IntakePosition.START) {
                pivotServo.setPosition(pivotServoStartPosition);
                //mode.sleep(50);
            }

            pivotServo.setPosition(pivotServoAimingPosition);
            fourBarServo.setPosition(fourBarServoAimingPosition);

            currentPosition = IntakePosition.AIM;
        }
    }

    public void MoveToIntakePosition()
    {
        if(currentPosition != IntakePosition.INTAKE) {

            //need to adjust the pivot if current position is START
            //otherwise, the pivot servo may block the four-bar servo
            if(currentPosition == IntakePosition.START) {
                pivotServo.setPosition(pivotServoStartPosition);
                //mode.sleep(50);
            }

            fourBarServo.setPosition(fourBarServoIntakePosition);
            pivotServo.setPosition(pivotServoIntakePosition);

            currentPosition = IntakePosition.INTAKE;
        }
    }

    public void MoveToOuttakePosition()
    {
        if(currentPosition != IntakePosition.OUTTAKE)
        {
            //need to adjust the pivot if current position is START
            //otherwise, the pivot servo may block the four-bar servo
            //if(currentPosition == IntakePosition.START ||
             //   currentPosition == IntakePosition.INTAKE) {
                pivotServo.setPosition(pivotServoOuttakePosition);
             //   mode.sleep(50);
            //}

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

    public double GetRotateServoPosition()
    {
        return rotateServo.getPosition();
    }

}

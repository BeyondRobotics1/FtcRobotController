package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public enum IntakeMode
    {
        IN,
        TRANSFER_ONLY,
        INTAKE_ONLY,
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

    final private DcMotor intakeMotor;
    final private DcMotor transferMotor;
    private Servo led;
    private LinearOpMode mode;

    int latchLimit = 3;


    public Intake(HardwareMap hardwareMap, LinearOpMode linearOpMode)
    {
        this.mode = linearOpMode;
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");

        //led = hardwareMap.get(Servo.class, "led");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Set the spinner to intake or outtake mode
     *      intakeMotor:    negative power - intake
     *                      positive power - outtake
     *      transferMotor:  negative power - outtake
     *                      positive power - intake
     *
     * @param intakeMode
     */
    public void setIntakeMode(org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake.IntakeMode intakeMode)
    {
        if(intakeMode == org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake.IntakeMode.IN)
        {
            intake(0.925);
        }
        else if(intakeMode == org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake.IntakeMode.TRANSFER_ONLY)
        {
            intakeMotor.setPower(0);
            transferMotor.setPower(-1);
        }
        else if(intakeMode == org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake.IntakeMode.INTAKE_ONLY)
        {
            intakeMotor.setPower(-0.925);
            transferMotor.setPower(0);
        }
        else if (intakeMode == org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake.IntakeMode.OUT) {

            //outtake(0.5);
            intakeMotor.setPower(0.3);
            transferMotor.setPower(-0.3);
        }
        else if (intakeMode == org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake.IntakeMode.FEED) {
            intake(1);
        }
        else if (intakeMode == org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake.IntakeMode.SLOW_FEED)
        {
            intake(0.65); //0.65
        }
        else if (intakeMode == org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake.IntakeMode.MEDIUM_FEED)
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
        double localPower = Math.abs(power);

        intakeMotor.setPower(-localPower);
        transferMotor.setPower(localPower);
    }

    public void intake(double intakePower,
                       double transferPower)
    {
        intakeMotor.setPower( -Math.abs(intakePower));
        transferMotor.setPower(Math.abs(transferPower));
    }

    //Power should be positive
    public void outtake(double power)
    {
        double localPower = Math.abs(power);
        intakeMotor.setPower(localPower);
        transferMotor.setPower(-localPower);
    }

    //TBD
    public int detectedArtifacts()
    {
        return 1;
    }


    public void setLedColor(double colorPosition)
    {
        if(led != null) {
            led.setPosition(colorPosition);
        }
    }
}

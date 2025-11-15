package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public enum IntakeMode
    {
        IN,
        VIN,
        HIN,
        OUT,
        FEED,
        IDLE
    }

    private DcMotor horizontalIntake;
    private DcMotor verticalIntake;
    private LinearOpMode mode;

    public Intake(HardwareMap hardwareMap, LinearOpMode linearOpMode)
    {
        this.mode = linearOpMode;
        horizontalIntake = hardwareMap.get(DcMotor.class, "horizontalIntake");
        verticalIntake = hardwareMap.get(DcMotor.class, "verticalIntake");
    }

    /**
     * Set the spinner to intake or outtake mode
     * @param intakeMode
     */
    public void SetIntakeMode(IntakeMode intakeMode)
    {
        if(intakeMode == Intake.IntakeMode.IN)
        {
            horizontalIntake.setPower(-1);
            verticalIntake.setPower(-1);
        }
        else if(intakeMode == Intake.IntakeMode.VIN)
        {
            horizontalIntake.setPower(0);
            verticalIntake.setPower(-1);
        }
        if(intakeMode == Intake.IntakeMode.HIN)
        {
            horizontalIntake.setPower(-1);
            verticalIntake.setPower(0);
        }
        else if (intakeMode == Intake.IntakeMode.OUT) {
            outtake(1);
        }
        else if (intakeMode == IntakeMode.FEED) {
            intake(-1);
        }
        else {
            intake(0);
        }
    }

    //power should be positive
    public void intake(double power)
    {
        double localPower = Math.abs(power);
        horizontalIntake.setPower(-localPower);
        verticalIntake.setPower(-localPower);
    }

    //power should be negative
    public void outtake(double power)
    {
        double localPower = Math.abs(power);
        horizontalIntake.setPower(localPower);
        verticalIntake.setPower(localPower);
    }

    public void stopVertical()
    {
        verticalIntake.setPower(0);
    }
}

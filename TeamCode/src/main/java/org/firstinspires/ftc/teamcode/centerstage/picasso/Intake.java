package org.firstinspires.ftc.teamcode.centerstage.picasso;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    LinearOpMode mode;
    DcMotorEx intake;

    enum IntakeMode
    {
        IN,
        OUT,
        IDLE
    }

    IntakeMode intakeMode;

    public Intake(HardwareMap hardwareMap, LinearOpMode mode)
    {
        this.mode = mode;

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMode = IntakeMode.IDLE;

    }

    //take in the pixels, work along with intake
    public void setPower(double power)
    {
        intake.setPower(power);

        if(power > 0.05)
            intakeMode = IntakeMode.IN;
        else if (power < -0.05)
            intakeMode = IntakeMode.OUT;
        else
            intakeMode = IntakeMode.IDLE;
    }

    public IntakeMode intakeMode()
    {
        return intakeMode;
    }
}


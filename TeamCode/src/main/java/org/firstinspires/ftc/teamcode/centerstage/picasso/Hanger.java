package org.firstinspires.ftc.teamcode.centerstage.picasso;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Hanger class to control the hanger mechanisms
 */
public class Hanger {

    //linear mode in case of telemetry
    LinearOpMode mode;

    //two servos to move up/down the two hooks
    Servo hanger1;
    Servo hanger2;

    //one motor to pull the string
    DcMotorEx motor;

    enum HookMode
    {
        HOOK_UP,
        HOOK_DOWN,
        HOOK_STAY
    }

    HookMode hookMode;

    //constructor
    public Hanger(HardwareMap hardwareMap, LinearOpMode mode) {
        this.mode = mode;

        hanger1 = hardwareMap.get(Servo.class, "hanger1");
        hanger2 = hardwareMap.get(Servo.class, "hanger2");

        hanger2.setDirection(Servo.Direction.REVERSE);

        motor = hardwareMap.get(DcMotorEx.class, "hanger");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //starting state of the hook
        hookMode = HookMode.HOOK_STAY;
    }

    public void HookUp()
    {
        if(hookMode != HookMode.HOOK_UP) {
            hanger1.setPosition(0);
            hanger2.setPosition(0);

            hookMode = HookMode.HOOK_UP;
        }
    }

    public void HookDown() {
        if (hookMode != HookMode.HOOK_DOWN)
        {
            hanger1.setPosition(1);
            hanger2.setPosition(1);

            hookMode = HookMode.HOOK_DOWN;
        }
    }

    public void HookStay()
    {
        if(hookMode != HookMode.HOOK_STAY) {
            hanger1.setPosition(0.5);
            hanger2.setPosition(0.5);

            hookMode = HookMode.HOOK_STAY;
        }
    }

    //Set the motor power to pull the two hooks
    public void SetMotorPower(double power)
    {
        motor.setPower(power);
    }
}

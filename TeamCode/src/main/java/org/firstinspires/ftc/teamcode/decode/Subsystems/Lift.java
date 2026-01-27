package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Lify subsystem
public class Lift {

    //hardware
    Clutch clutch;
    SlideHolder holder;

//    //Use DcMotorEx to support bulk read.
//    DcMotorEx motorFrontLeft;
//    DcMotorEx motorBackLeft;
//    DcMotorEx motorFrontRight;
//    DcMotorEx motorBackRight;

    //
    public Lift(HardwareMap hardwareMap)
    {
        clutch = new Clutch(hardwareMap);
        holder = new SlideHolder(hardwareMap);

//        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
//        motorBackLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
//        motorBackRight = hardwareMap.get(DcMotorEx.class, "rightBack");
//        motorFrontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
//
//        //Reverse motors
//        //motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        //motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
//        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //
        clutch.disengage();
        holder.hold();
    }

    //engage clutch or not
    public void engageClutch(boolean tf)
    {
        if(tf)
            clutch.engage();
        else
            clutch.disengage();
    }

    public void releaseHolder(boolean tf)
    {
        if(tf)
            holder.release();
        else
            holder.hold();
    }

//    public void liftUp(double power)
//    {
////        motorFrontLeft.setPower(Math.abs(power));
////        motorFrontRight.setPower(Math.abs(power));
////
////        motorBackLeft.setPower(-Math.abs(power));
////        motorBackRight.setPower(-Math.abs(power));
//    }

    public double getLeftClutchServoPosition()
    {
        return clutch.getLeftClutchServoPosition();
    }

    public double getRightClutchServoPosition()
    {
        return clutch.getRightClutchServoPosition();
    }

    public double getLeftSlideServoPosition()
    {
        return holder.getLeftSlideServoPosition();
    }

    public double getRightSlideServoPosition()
    {
        return holder.getRightSlideServoPosition();
    }
}

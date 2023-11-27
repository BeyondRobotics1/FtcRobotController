package org.firstinspires.ftc.teamcode.centerstage.picasso;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.Helper;

public class PicassoSlide{

    static final double     COUNTS_PER_MOTOR_REV    = 537.7;//Rev Motor: 10.43 gear ratio; Gobilda 537.7
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;   // No External Gearing.
    static final double     PULLEY_DIAMETER_INCHES  = 1.404 ; // spool wheel inches
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (PULLEY_DIAMETER_INCHES * 3.1415);

    DcMotorEx slide1;
    DcMotorEx slide2;

    LinearOpMode mode;

    public PicassoSlide(HardwareMap hardwareMap, LinearOpMode mode) {
        this.mode = mode;

        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
    }

    public void setPower(double power)
    {
        double localPower = Helper.squareWithSign(power);//Helper.cubicWithSign(power);//

//        //slide move down
//        if(localPower < -0.01) {
//            //low limit touch sensor is pushed
//            if (!touchSensorLowLimit.getState()) {
//                localPower = 0;
//
//                //reset the slide 0 position
//                //we want to do this only once
//                if(!resetSlideDone)
//                {
//                    runWithEncoder();
//                    resetSlideDone = true;
//                }
//            }
//        }
//        else if(localPower > 0.01) { //slide move up
//            //high limit touch sensor is pushed
//            if (!touchSensorHighLimit.getState())
//                localPower = 0;
//        }

        slide1.setPower(localPower);
        slide2.setPower(localPower);

    }
}

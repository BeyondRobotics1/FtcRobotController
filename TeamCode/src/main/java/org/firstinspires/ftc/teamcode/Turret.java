package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    //turret position, 0-left, 1 - middle, 2 - right
    int turretPosition = 1;

    //The left, middle, and right position for arm servo
    //0.265, 0.165, 0.065
    double turretServoPositions[] = {0.267, 0.167, 0.063};

    Servo servoTurret;
    Slide slide;

    public Turret(HardwareMap hardwareMap, Slide slide){
        servoTurret = hardwareMap.get(Servo.class,"arm");
        this.slide = slide;
    }

    /**
     * Set turret to left, middle, or right
     * @param turretPosition: 0 - left, 1 - middle, 2 -right
     */
    public void setPosition(int turretPosition)
    {
        //move to a different position
        if (this.turretPosition != turretPosition) {

            //moving to middle
            if (turretPosition == 1 && this.turretPosition == 0)
                servoTurret.setPosition(0.165);
            else
                servoTurret.setPosition(turretServoPositions[turretPosition]);

            this.turretPosition = turretPosition;

        }
    }

    /**
     * Set turret to left, middle, or right
     * @param turretPosition: 0 - left, 1 - middle, 2 -right
     */
    public void setPositionCheckSlideHeight(int turretPosition)
    {
        //move to a different position
        if (this.turretPosition != turretPosition) {

            //make sure the slide not in the low positions
            if(slide.getSlideHeightInches() > 13)
            {
                //moving to middle
                if (turretPosition == 1 && this.turretPosition == 0)
                    servoTurret.setPosition(0.165);
                else
                    servoTurret.setPosition(turretServoPositions[turretPosition]);

                this.turretPosition = turretPosition;
            }
        }
    }

    /**
     * set to center poistion, useful when auto/tele op inits
     */
    public void setToCenterPosition()
    {
        int turretPosition = 1;
        servoTurret.setPosition(0.165);
    }

    public double getTurretPosition(){
        return servoTurret.getPosition();
    }
}

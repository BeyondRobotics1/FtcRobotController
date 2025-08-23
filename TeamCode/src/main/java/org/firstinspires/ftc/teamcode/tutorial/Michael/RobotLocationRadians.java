package org.firstinspires.ftc.teamcode.tutorial.Michael;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Robot Location Radians", group = "Michael")
@Disabled
public class RobotLocationRadians extends OpMode {
    double angleRadians;

    public RobotLocationRadians(double angelDegrees){
        this.angleRadians = Math.toRadians(angelDegrees);
    }

    public double getHeading(){
        double angle = this.angleRadians;
        while (angle > Math.PI){
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI){
            angle += 2 * Math.PI;
        }
        return Math.toDegrees(angle);
    }

    @Override
    public String toString(){
        return "RobotLocationRadians: angle" + angleRadians + ")";
    }
    public void turn (double angleChangeDegrees){
        angleRadians += Math.toRadians(angleChangeDegrees);
    }
    public void setAngle(double angleDegrees){
        this.angleRadians = Math.toRadians(angleDegrees);
    }


    @Override
    public void init() {

    }

    @Override
    public void loop(){

    }
}

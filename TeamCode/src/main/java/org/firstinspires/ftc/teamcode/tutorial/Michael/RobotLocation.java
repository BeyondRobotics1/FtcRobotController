package org.firstinspires.ftc.teamcode.tutorial.Michael;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Robot Location", group = "Michael")
@Disabled
public class RobotLocation extends OpMode {
    double angle;

    public RobotLocation(double angle){
        this.angle = angle;
    }

    public double getHeading(){
        double angle = this.angle;
        while (angle > 180){
            angle -= 360;
        }
        while (angle < -180){
            angle += 360;
        }
        return angle;
    }

    @Override
    public String toString(){
        return "Robot Location: angle (" + angle + ")";
    }
    public void turn (double angleChange){
        angle += angleChange;
    }
    public void setAngle(double angle){
        this.angle = angle;
    }


    @Override
    public void init() {

    }

    @Override
    public void loop(){

    }
}

package org.firstinspires.ftc.teamcode.tutorial.Michael;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name = "Distance and Color Sensor", group = "Michael")
@Disabled
public class DistanceColorSensor extends OpMode {
    ProgrammingBoard1 distanceColorSensor = new ProgrammingBoard1();

    @Override
    public void init(){
        distanceColorSensor.init(hardwareMap);
    }

    @Override
    public void loop(){
        int gain = 0;
        String colorString = "Red";

        if(gamepad1.a){
            gain = 0;
        }
        else if(gamepad1.b){
            gain = 1;
            colorString = "Blue";
        }
        else{
            gain = 2;
            colorString = "Green";
        }

        telemetry.addData(colorString, distanceColorSensor.getColor(gain));
        telemetry.addData("Distance (CM)", distanceColorSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance (IN)", distanceColorSensor.getDistance(DistanceUnit.INCH));
    }
}

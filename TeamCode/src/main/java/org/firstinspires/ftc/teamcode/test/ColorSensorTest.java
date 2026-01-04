package org.firstinspires.ftc.teamcode.test;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(group = "Test")
//@Disabled

public class ColorSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        NormalizedColorSensor colorSensor = null;
        //hardwareMap.get(NormalizedColorSensor.class, "color2");

        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        // for an explanation of HSV color.
        final float[] hsvValues = new float[3];



        telemetry.addData("gamepad1.a", "Top Color Sensor." );
        telemetry.addData("gamepad1.b", "Bottom Color Sensor." );
        telemetry.addData("gamepad1.y", "Front Color Sensor." );
        telemetry.update();

        String name = "";

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            if(colorSensor == null)
            {
                if (gamepad1.a) {
                    colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color2");
                    name = "Top Color Sensor";

                    // If possible, turn the light on in the beginning (it might already be on anyway,
                    // we just make sure it is if we can).
                    if (colorSensor instanceof SwitchableLight) {
                        ((SwitchableLight)colorSensor).enableLight(true);
                    }
                }
                else if (gamepad1.b) {
                    colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color1");
                    name = "Bottom Color Sensor";

                    // If possible, turn the light on in the beginning (it might already be on anyway,
                    // we just make sure it is if we can).
                    if (colorSensor instanceof SwitchableLight) {
                        ((SwitchableLight)colorSensor).enableLight(true);
                    }
                }
                else if (gamepad1.y) {
                    colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color0");
                    name = "Bottom Color Sensor";

                    // If possible, turn the light on in the beginning (it might already be on anyway,
                    // we just make sure it is if we can).
                    if (colorSensor instanceof SwitchableLight) {
                        ((SwitchableLight)colorSensor).enableLight(true);
                    }
                }
            }


            if(colorSensor != null) {

                colorSensor.setGain(2);

                telemetry.addData("Sensor Name", name);

                // Get the normalized colors from the sensor
                NormalizedRGBA colors = colorSensor.getNormalizedColors();

                /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
                 * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
                 * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
                 * for an explanation of HSV color. */

                // Update the hsvValues array by passing it to Color.colorToHSV()
                Color.colorToHSV(colors.toColor(), hsvValues);

                telemetry.addLine()
                        .addData("Red", "%.3f", colors.red)
                        .addData("Green", "%.3f", colors.green)
                        .addData("Blue", "%.3f", colors.blue);
                telemetry.addLine()
                        .addData("Hue", "%.3f", hsvValues[0])
                        .addData("Saturation", "%.3f", hsvValues[1])
                        .addData("Value", "%.3f", hsvValues[2]);
                telemetry.addData("Alpha", "%.3f", colors.alpha);

                if (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < 4) { //
                    if (hsvValues[0] >= 120 && hsvValues[0] <= 200)
                        telemetry.addData("Color", "%s", "Green");
                    else if (hsvValues[0] > 200 && hsvValues[0] < 250)
                        telemetry.addData("Color", "%s", "Purple");
                } else
                    telemetry.addData("Color", "%s", "White");


//            if(((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < 5) { //
//                if (hsvValues[0] >= 50 && hsvValues[0] < 92)
//                    telemetry.addData("Color", "%s", "Yellow");
//                else if ((hsvValues[0] >= 0 && hsvValues[0] <= 20) ||
//                        (hsvValues[0] >= 340 && hsvValues[0] <= 360))
//                    telemetry.addData("Color", "%s", "Red");
//                else if (hsvValues[0] >= 210 && hsvValues[0] <= 270)
//                    telemetry.addData("Color", "%s", "Blue");
//                else if (hsvValues[0] >= 120 && hsvValues[0] <= 150)
//                    telemetry.addData("Color", "%s", "Green");
//                else if (hsvValues[0] > 200 && hsvValues[0] < 230)
//                    telemetry.addData("Color", "%s", "Purple");
//            }
//            else
//                telemetry.addData("Color", "%s", "White");

                /* If this color sensor also has a distance sensor, display the measured distance.
                 * Note that the reported distance is only useful at very close range, and is impacted by
                 * ambient light and surface reflectivity. */
                if (colorSensor instanceof DistanceSensor) {
                    telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
                }
            }

            telemetry.update();
        }
    }

}

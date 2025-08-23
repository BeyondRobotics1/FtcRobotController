package org.firstinspires.ftc.teamcode.tutorial.Michael;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Array OpMode", group = "Michael")
@Disabled
public class ArrayOpMode extends OpMode {
    String[] words = {"Zeroth", "First", "Second", "Third", "Fourth", "Fifth", "Infinity"};
    int wordIndex;
    double delay_secs = 1.0;

    double nextTime = 1;

    @Override
    public void init(){
        wordIndex = 0;
    }

    @Override
    public void loop(){
        if(nextTime <= getRuntime()){
            wordIndex++;
            if(wordIndex >= words.length){
                wordIndex = words.length - 1;
            }
            nextTime = getRuntime() + delay_secs;
        }
        telemetry.addLine(words[wordIndex]);
        if(wordIndex == 6){
            terminateOpModeNow();
        }
    }
}

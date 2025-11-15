package org.firstinspires.ftc.teamcode.tutorial.Michael;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Limelight", group = "Michael")
@Disabled
public class SimpleLimelightOpMode extends OpMode {
    Limelight3A limelight3A;

    @Override
    public void init(){
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void start(){
        limelight3A.pipelineSwitch(0);
        limelight3A.start();
    }

    @Override
    public void loop(){
        LLResult llResult = limelight3A.getLatestResult();
        if(llResult != null && llResult.isValid()){
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
        } else {
            telemetry.addLine("None found");
        }
    }
}

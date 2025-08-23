package org.firstinspires.ftc.teamcode.tutorial.Michael;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous()
@Disabled
public class AutoState1 extends OpMode {
    ProgrammingBoard1 autoState = new ProgrammingBoard1();
    int state;

    @Override
    public void init(){
        autoState.init(hardwareMap);
    }

    @Override
    public void start(){
        state = 0;
    }

    @Override
    public void loop(){
        telemetry.addData("State", state);
        if(state == 0){
            autoState.setServoPosition(0.5);
            state = 1;
            telemetry.addData("State", state);
        }
        else if(state == 1){
            autoState.setServoPosition(0.0);
            if(autoState.isPressed()){
                state = 2;
                telemetry.addData("State", state);
            }
        }
        else if(state == 2){
            autoState.setServoPosition(1.0);
            autoState.setMotor(0.5);
            telemetry.addData("Angle", autoState.getHeading(AngleUnit.DEGREES));
            if(autoState.getHeading(AngleUnit.DEGREES) > 45){
                state = 3;
                telemetry.addData("State", state);
            }
        }
        else if(state == 3){
            autoState.setMotor(0.0);
            state = 4;
            telemetry.addData("State", state);
        } else {
            telemetry.addData("Auto", "Finished");
        }
    }
}

package org.firstinspires.ftc.teamcode.tutorial.Michael;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
@Disabled
public class AutoState2 extends OpMode {
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
        switch(state){
            case 0:
                autoState.setServoPosition(0.5);
                if(autoState.isPressed()){
                    state = 1;
                }
                break;
            case 1:
                autoState.setServoPosition(0.0);
                if(!autoState.isPressed()){
                    state = 2;
                }
                break;
            case 2:
                autoState.setServoPosition(1.0);
                autoState.setMotor(0.5);
                if(autoState.getHeading(AngleUnit.DEGREES) > 45){
                    state = 3;
                }
                break;
            case 3:
                autoState.setMotor(0.0);
                state = 4;
                break;
            default:
                telemetry.addData("Auto", "Finished");
        }
    }
}

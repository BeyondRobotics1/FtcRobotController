package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp
public class Test extends OpMode {

    private TestRobot robot;

    @Override
    public void init(){

        telemetry.addData("Init", "start");
        int teamNumber = 21380;

        telemetry.addData("Hello Team", teamNumber);

        telemetry.addData("Init", "end");

        robot = new TestRobot(hardwareMap);
        robot.init();

    }

    @Override
    public void loop(){

        telemetry.addData("Left stick x:", gamepad1.left_stick_x);
        telemetry.addData("Left stick y:", gamepad1.left_stick_y);
        telemetry.addData("A button:", gamepad1.a);

        telemetry.addData("left stick x - right stick x", gamepad1.left_stick_x - gamepad1.right_stick_x);

        telemetry.addData("touch sensor", robot.getTouchSensorState());

        telemetry.addData("color red", robot.getAmountRed());
        telemetry.addData("color blue", robot.getAmountBlue());

        telemetry.addData("distance sensor", robot.getDistanceCM());

        telemetry.addData("motor turns", robot.getMotorTurns());


        double speed = gamepad1.left_stick_y*-1.0;
        robot.setMotorSpeed(speed);
        telemetry.addData("speed", speed);

        if(gamepad1.a){
            robot.setServoPosition(1.0);
        }
//        else if (gamepad1.b){
//            robot.setServoPosition(0.0);
//        }
        else{
            robot.setServoPosition(0.0);
        }

        double speedSlideMotor = gamepad1.right_stick_y*-1.0;
        robot.setSlideMotorSpeed(speedSlideMotor);
        telemetry.addData("Slide speed", speedSlideMotor);

    }


}
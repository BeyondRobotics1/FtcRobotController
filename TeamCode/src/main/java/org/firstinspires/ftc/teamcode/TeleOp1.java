package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOp1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //drive train
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);

        //Arm arm = new Arm(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            /*
            //claw
            if (gamepad2.left_bumper) {
                arm.openClaw();
            } else if (gamepad2.right_bumper) {
                arm.closeClaw();
            }

            //arm
            if (gamepad2.y) {
                arm.setClawPosition(0);
            } else if (gamepad2.b) {
                arm.setClawPosition(1);
            } else if (gamepad2.a) {
                arm.setClawPosition(2);
            }

            arm.moveSlide(-gamepad2.left_stick_y);
*/

            //drive train
            driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }
}

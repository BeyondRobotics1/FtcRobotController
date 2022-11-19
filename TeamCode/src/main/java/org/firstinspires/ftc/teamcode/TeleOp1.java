package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOp1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //drive train
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);

        Arm arm = new Arm(hardwareMap);

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
*/
            //arm
            if (gamepad2.y) {
                arm.setClawPosition(1);
            } else if (gamepad2.b) {
                arm.setClawPosition(2);
            } else if (gamepad2.x) {
                arm.setClawPosition(0);
            }

            arm.moveSlide(-gamepad2.left_stick_y);

            /*
            if (gamepad2.y) {
                arm.setTurretPosition(0.165);
            } else if (gamepad2.x) {
                arm.setTurretPosition(0.265);
            } else if (gamepad2.b) {
                arm.setTurretPosition(0.065);
            }
             */

            telemetry.addLine(String.format("\nDetected turret position=%f", arm.getTurretPosition()));


            //drive train
            driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.update();
        }
    }
}

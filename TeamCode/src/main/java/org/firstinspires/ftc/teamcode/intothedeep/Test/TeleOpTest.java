package org.firstinspires.ftc.teamcode.intothedeep.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;


/**
 * This is an example teleop that showcases movement and control of two servos and field-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@TeleOp(name = "TeleOpTest", group = "Into the Deep")
public class TeleOpTest extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    /** This method is call once when init is played, it initializes the follower and subsystems **/
    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: false
        */


        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();
        if (gamepad1.left_bumper){
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
        }


        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Telemetry Outputs of our ClawSubsystem */

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.88)//30.6lb(13.88kg) 25.8b (11.7kg) -> 30 lb (13.6kg)
            .forwardZeroPowerAcceleration(-27.66924866)//-27.0643
            .lateralZeroPowerAcceleration(-62.28069348)//-57.9476
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.2,
                    0,
                    0.01,
                    0.02

            ))//(0.06, 0, 0.01, 0.03)

//0.09, 0, 0,01, 0.03
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.8,
                    0,
                    0,
                    0.01
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    2.5,
                    0,
                    0.1,
                    0.0005
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.1,
                    0,
                    0.00035,
                    0.6,
                    0.015
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02,
                    0,
                    0.000005,
                    0.6,
                    0.01
            ))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0005);

    //maxPower 0.95
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.998)
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftBack")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)//FORWARD
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)//Forward
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)//Reverse
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)//Reverse
            .xVelocity(71.64968496)//73.48
            .yVelocity(56.26285967);//57.73

    //pedro and gobilda pinpoint tester has X, Y offset reversed
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(118)//106
            .strafePodX(-143.6)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    /**
     These are the PathConstraints in order:
     tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint, timeoutConstraint,
     brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart

     The BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10 and shouldn't be changed.
     */

    /*
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            0.75,
            10,
            1
    );
     */

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            0.75,
            10,
            1
    );

    //Add custom localizers or drivetrains here
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
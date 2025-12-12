package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants
{
    // Values from auto, PID, and centripetal tuners
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(13.6078)
        .lateralZeroPowerAcceleration(-62.18029)
        .forwardZeroPowerAcceleration(-32.5755)
        .translationalPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(0.1, 0, 0.01, 0))
        .headingPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(1,0,0.03, 0))
        .drivePIDFCoefficients(new com.pedropathing.control.FilteredPIDFCoefficients(0.025, 0,0.0003, 0.6, 0.01));



    // What conditions a path ends
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,

            1.5,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap)
    {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                
                .pinpointLocalizer(localizerConstants)

                .build();
    }

    // Drivetrain constants, motor names, motor directions
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(62.5389)
            .yVelocity(49.3334)


            .rightFrontMotorName("right_front_drive")
            .rightRearMotorName("right_back_drive")
            .leftRearMotorName("left_back_drive")
            .leftFrontMotorName("left_front_drive")

            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(4.625)
            .strafePodX(-2.375)

            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint") // CHANGE LATER

            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
}

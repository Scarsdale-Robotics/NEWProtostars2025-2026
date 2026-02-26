package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Configurable
//TODO: Tune all
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()

            .forwardZeroPowerAcceleration(-37)
            .lateralZeroPowerAcceleration(-62.23)
            .mass(10.56)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.055,
                    0,
                    0.001,
                    0.015
            ))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.4,
                    0.00001,
                    0.05,
                    0.0004
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.8,
                    0,
                    0.005,
                    0.02
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    2.5,
                    0,
                    0.1,
                    0.0005
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.032,
                    0,
                    0.000025,
                    0.6,
                    0.0
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.015,
                    0,
                    0.000005,
                    0.6,
                    0.015
            ))
            .useSecondaryDrivePIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryTranslationalPIDF(false)
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0002);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.1, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(2)
            .strafePodX(-3.1)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pp")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.85)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("leftFront")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("rightBack")
            //check these
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .xVelocity(45.18)
            .yVelocity(33);
    //TODO: Tune
}

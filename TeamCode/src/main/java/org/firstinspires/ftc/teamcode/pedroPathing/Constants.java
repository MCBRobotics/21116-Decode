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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Central configuration for Pedro Pathing autonomous navigation.
 * Tune constants here rather than scattering magic numbers across OpModes.
 */
public class Constants {

    // Physical properties of the robot — used by Pedro to model how it decelerates
    // mass: robot weight in kg (affects how aggressively the follower corrects)
    // forwardZeroPowerAcceleration / lateralZeroPowerAcceleration: measured deceleration (in/s²)
    // when motors are set to 0 — run the tuning OpMode to get accurate values
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5) // TODO insert
            .forwardZeroPowerAcceleration(10) // TODO insert
            .lateralZeroPowerAcceleration(10); // TODO insert

    // Drivetrain configuration — motor names must match the Robot Controller hardware config
    // xVelocity / yVelocity: max wheel velocities (in/s) used to scale path following speeds
    // Left-rear is reversed because it physically spins the opposite way for forward motion
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(.7)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rb")
            .leftRearMotorName("lb")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(20) // tuned with a 10-inch test: repeat at higher distance for accuracy
            .yVelocity(20); // TODO: lateral velocity tuning

    // Path motion limits: (maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration)
    // Lower maxVelocity (0.99 ≈ full speed) keeps the path smooth; increase acceleration carefully
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    // Pinpoint odometry pod offsets from the robot's center of rotation, in inches
    // forwardPodY: how far the forward-measuring pod is offset laterally from center
    // strafePodX:  how far the strafe-measuring pod is offset forward from center
    // Getting these wrong causes the robot to arc instead of drive straight during auto
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1)
            .strafePodX(-1)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint") //TODO: rename to match hardware names
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    /** Builds and returns a fully configured Pedro Pathing Follower for use in autonomous OpModes. */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}

package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name = "NewAuto", group = "Auto")
public class NewAuto extends OpMode {

    /* DECLARE MOTORS */
    private final DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "leftfrontdrive");
    private final DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "leftbackdrive");
    private final DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfrontdrive");
    private final DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rightbackdrive");

    /* DECLARE POSES */
    private final Pose startingPose = new Pose(72, 72, Math.toRadians(90));
    private final Pose endingPose = new Pose(72, 60, Math.toRadians(270));

    /* DECLARE PATHs */
    private final Path firstPath = new Path(new BezierLine(startingPose, endingPose));

    private final Follower follower = Constants.createFollower(hardwareMap);

    @Override
    public void init() {
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        follower.setStartingPose(startingPose);
    }

    @Override
    public void loop() {
        follower.followPath(firstPath);
    }
}

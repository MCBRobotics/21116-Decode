package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous (name = "NewAuto", group = "Auto")
public class NewAuto extends OpMode {

    DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "leftfrontdrive");
    DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "leftbackdrive");
    DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfrontdrive");
    DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rightbackdrive");



    Follower follower = Constants.createFollower(hardwareMap);

    public void init() {
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        Pose startingPose = new Pose(72, 72, Math.toRadians(180));

        follower.setStartingPose(startingPose);
    }

    public void loop() {
    }
}

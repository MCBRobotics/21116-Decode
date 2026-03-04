package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name = "DecodeAuto", group = "Autonomous")
public class AutoTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private DcMotor rotate, intake, shooter;
    private Servo blocker, pusher, hood;
    private Limelight3A limelight;
    private final Pose scorePose = new Pose(64, 80, Math.toRadians(140)); // where we shoot
    private final Pose loadPose = new Pose(10, 9, Math.toRadians(180)); // blue loading zone

    private final Pose pickup1Pose = new Pose(23, 84.85, Math.toRadians(180)); // row of balls closest to goal
    private final Pose pickup2Pose = new Pose(23, 60, Math.toRadians(180)); // middle row
    private final Pose pickup3Pose = new Pose(23, 35, Math.toRadians(180)); // row closest to loading zone


    private Path start_path;
    private PathChain load_path, pickup_path1, pickup_path2, pickup_path3;

    private boolean BlueAlliance;

    // We need to be able to navigate our side without bumping into the other team.
    // These paths should move the robot in a counter-clockwise rotation.
    // Inevitably, we'll need to change and update the paths and state machine to suit the alliance.

    Runnable shoot_shooter = () -> { // callbacks in the paths require "Runnables"
        double currentTime = opmodeTimer.getElapsedTimeSeconds();
        while (!(opmodeTimer.getElapsedTimeSeconds() > currentTime + 2)) {
            shooter.setPower(1.0);
        }
        pusher.setPosition(1.0);
        pusher.setPosition(0.0);
        shooter.setPower(0.0);
    };

    private void createPaths() {

        load_path = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(72, 30), loadPose)) // moves to loading zone from scoring position
                .setLinearHeadingInterpolation(scorePose.getHeading(), loadPose.getHeading())
                .addPath(new BezierCurve(follower.getPose(), new Pose(72, 30), scorePose)) // moves back to scoring position
                .setLinearHeadingInterpolation(loadPose.getHeading(), scorePose.getHeading())
                .build();

        pickup_path1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,  pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addParametricCallback(
                        0.3,
                        () -> intake.setPower(-0.8)
                )
                .addParametricCallback(
                        1.0,
                        () -> intake.setPower(0)
                )
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(1.0, shoot_shooter)
                .build();

        pickup_path2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(65, 52), pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addParametricCallback(
                        0.3,
                        () -> intake.setPower(-0.8)
                )
                .addParametricCallback(
                        1.0,
                        () -> intake.setPower(0)
                )
                .addPath(new BezierCurve(pickup2Pose, new Pose(65, 52), scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(1.0, shoot_shooter)
                .build();

        pickup_path3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,  new Pose(75, 30), pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .addParametricCallback(
                        0.3,
                        () -> intake.setPower(-0.8)
                )
                .addParametricCallback(
                        1.0,
                        () -> intake.setPower(0)
                )
                .addPath(new BezierCurve(pickup3Pose, new Pose(75, 30), scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(1.0, shoot_shooter)
                .build();
    }

    // TODO update config
    private void defineMechanisms() {
        rotate  = hardwareMap.get(DcMotor.class, "rotate");
        intake  = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        hood = hardwareMap.get(Servo.class, "hood");
        blocker  = hardwareMap.get(Servo.class, "blocker");
        pusher  = hardwareMap.get(Servo.class, "pusher");

        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private boolean pickedup1 = false;
    private boolean pickedup2 = false;
    private boolean pickedup3 = false;

    // The state machine may change with strategy:
    // For example, the other robot may pickup one row of balls before you do
    // DO NOT CHANGE THE PATHS only change the decision flows here.
    // TODO: add the red alliance paths
    public void PathUpdate() { // state machine
        switch (pathState) {
            case 0:
                follower.followPath(load_path);
                if (!pickedup1) {
                    changePath(1);
                }
                if (!pickedup2) {
                    changePath(2);
                }
                if (!pickedup3) {
                    changePath(3);
                }
            case 1:
                follower.followPath(pickup_path1);
                pickedup1 = true;
                changePath(2);
                break;
            case 2:
                follower.followPath(pickup_path2);
                pickedup2 = true;
                break;
            case 3:
                follower.followPath(pickup_path3);
                pickedup3 = true;
                break;
        }
    }

    public void changePath(int change) {
        pathState = change;
        pathTimer.resetTimer();
    }

    // Called once when we hit INIT
    // TODO add option at init to choose alliance
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        defineMechanisms();
        createPaths();

    }

    public void init_loop() {
        if (gamepad1.left_bumper) {
            BlueAlliance = true;
        } else if (gamepad1.right_bumper) {
            BlueAlliance = false;
        }
    }

    // Called once when we hit PLAY
    public void start() {
        opmodeTimer.resetTimer();
        changePath(0);
    }

    // LOOPS as the op mode is running
    public void loop() {
        follower.update();
        PathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("time left", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}

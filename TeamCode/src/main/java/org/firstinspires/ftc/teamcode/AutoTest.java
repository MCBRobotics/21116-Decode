package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name = "Decode Auto", group = "Autonomous")
public class AutoTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private DcMotor rotate, intake, shooter;
    private Servo blocker, pusher, hood;
    private Limelight3A limelight;
    private final Pose scorePose = new Pose(64, 80, Math.toRadians(140)); // where we shoot
    private final Pose loadPose = new Pose(10, 9, Math.toRadians(180)); // blue loading zone
    private final Pose pickup1Pose = new Pose(23, 84.85, Math.toRadians(180)); // row of balls closest to goal
    private final Pose pickup2Pose = new Pose(23, 60, Math.toRadians(180)); // middle row
    private final Pose pickup3Pose = new Pose(23, 35, Math.toRadians(180)); // row closest to loading zone
    private final Pose red_Score = new Pose(80, 80, Math.toRadians(40)); // where we shoot red side
    private final Pose red_loadPose = new Pose(140, 5, Math.toRadians(0)); // red loading zone
    private final Pose red_pickup1Pose = new Pose(121, 84.85, Math.toRadians(0));
    private final Pose red_pickup2Pose = new Pose(121, 60, Math.toRadians(0));
    private final Pose red_pickup3Pose = new Pose(121, 35, Math.toRadians(0));
    private PathChain load_path, pickup_path1, pickup_path2, pickup_path3, red_loadPath, redpickup_path1, redpickup_path2, redpickup_path3;


    // We need to be able to navigate our side without bumping into the other team.
    // These paths should move the robot in a counter-clockwise rotation.
    // Inevitably, we'll need to change and update the paths and state machine to suit the alliance.
    // TODO change shoot_shooter runnable
    // This runnable and path callbacks are to avoid blocking threads
    Runnable shoot_shooter = () -> {
      shooter.setPower(1.0);
    };

    private void createPaths() {
        load_path = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(72, 30), loadPose)) // moves to loading zone from scoring position
                .setLinearHeadingInterpolation(scorePose.getHeading(), loadPose.getHeading())
                .addParametricCallback(1.0, () -> intake.setPower(-0.8))
                .addPath(new BezierCurve(follower.getPose(), new Pose(72, 30), scorePose)) // moves back to scoring position
                .setLinearHeadingInterpolation(loadPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.2, () -> intake.setPower(0))
                .addParametricCallback(1.0, shoot_shooter)
                .build();

        red_loadPath = follower.pathBuilder()
                .addPath(new BezierCurve(red_Score, new Pose(72, 30), red_loadPose))
                .setLinearHeadingInterpolation(red_Score.getHeading(), red_loadPose.getHeading())
                .addParametricCallback(1.0, () -> intake.setPower(-0.8))
                .addPath(new BezierCurve(red_loadPose, new Pose(72,30), red_Score))
                .setLinearHeadingInterpolation(red_loadPose.getHeading(), red_Score.getHeading())
                .addParametricCallback(0.2, () -> intake.setPower(0))
                .addParametricCallback(1.0, shoot_shooter)
                .build();

        pickup_path1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,  pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addParametricCallback(0.3, () -> intake.setPower(-0.8))
                .addParametricCallback(1.0, () -> intake.setPower(0))
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(1.0, shoot_shooter)
                .build();

        redpickup_path1 = follower.pathBuilder()
                .addPath(new BezierLine(red_Score,  red_pickup1Pose))
                .setLinearHeadingInterpolation(red_Score.getHeading(), red_pickup1Pose.getHeading())
                .addParametricCallback(0.3, () -> intake.setPower(-0.8))
                .addParametricCallback(1.0, () -> intake.setPower(0))
                .addPath(new BezierLine(red_pickup1Pose, red_Score))
                .setLinearHeadingInterpolation(red_pickup1Pose.getHeading(), red_Score.getHeading())
                .addParametricCallback(1.0, shoot_shooter)
                .build();

        pickup_path2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(65, 52), pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addParametricCallback(0.3, () -> intake.setPower(-0.8))
                .addParametricCallback(1.0, () -> intake.setPower(0))
                .addPath(new BezierCurve(pickup2Pose, new Pose(65, 52), scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(1.0, shoot_shooter)
                .build();

        redpickup_path2 = follower.pathBuilder()
                .addPath(new BezierCurve(red_Score, new Pose(79, 52), red_pickup2Pose))
                .setLinearHeadingInterpolation(red_Score.getHeading(), red_pickup2Pose.getHeading())
                .addParametricCallback(0.3, () -> intake.setPower(-0.8))
                .addParametricCallback(1.0, () -> intake.setPower(0))
                .addPath(new BezierCurve(red_pickup2Pose, new Pose(79, 52), red_Score))
                .setLinearHeadingInterpolation(red_pickup2Pose.getHeading(), red_Score.getHeading())
                .addParametricCallback(1.0, shoot_shooter)
                .build();

        pickup_path3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,  new Pose(75, 30), pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .addParametricCallback(0.3, () -> intake.setPower(-0.8))
                .addParametricCallback(1.0, () -> intake.setPower(0))
                .addPath(new BezierCurve(pickup3Pose, new Pose(75, 30), scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(1.0, shoot_shooter)
                .build();

        redpickup_path3 = follower.pathBuilder()
                .addPath(new BezierCurve(red_Score,  new Pose(69, 30), red_pickup3Pose))
                .setLinearHeadingInterpolation(red_Score.getHeading(), red_pickup3Pose.getHeading())
                .addParametricCallback(0.3, () -> intake.setPower(-0.8))
                .addParametricCallback(1.0, () -> intake.setPower(0))
                .addPath(new BezierCurve(red_pickup3Pose, new Pose(69, 30), red_Score))
                .setLinearHeadingInterpolation(red_pickup3Pose.getHeading(), red_Score.getHeading())
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
    public void PathUpdate() { // state machine
        if (BlueAlliance) {
            switch (pathState) {
                case 0:
                    if (!follower.isBusy()) {
                        follower.followPath(load_path);
                    }
                    if (!follower.isBusy()) {
                        if (!pickedup1) {
                            changePath(1);
                        }
                        if (!pickedup2) {
                            changePath(2);
                        }
                        if (!pickedup3) {
                            changePath(3);
                        }
                    }
                    break;
                case 1:
                    follower.followPath(pickup_path1);
                    pickedup1 = true;
                    if (!follower.isBusy()) {
                        changePath(2);
                    }
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
        } else {
            switch (pathState) {
                case 0:
                    follower.followPath(red_loadPath);
                    if (!follower.isBusy()) {
                        if (!pickedup1) {
                            changePath(1);
                        }
                        if (!pickedup2) {
                            changePath(2);
                        }
                        if (!pickedup3) {
                            changePath(3);
                        }
                    }
                    break;
                case 1:
                    follower.followPath(redpickup_path1);
                    pickedup1 = true;
                    if (!follower.isBusy()) {
                        changePath(2);
                    }
                    break;
                case 2:
                    follower.followPath(redpickup_path2);
                    pickedup2 = true;
                    break;
                case 3:
                    follower.followPath(redpickup_path3);
                    pickedup3 = true;
                    break;
            }
        }
    }

    public void changePath(int change) {
        pathState = change;
        pathTimer.resetTimer();
    }

    // Called once when we hit INIT
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        defineMechanisms();
        createPaths();
    }

    private boolean BlueAlliance = true;

    @Override
    public void init_loop() {
        if (gamepad1.left_bumper) {
            BlueAlliance = true;
            follower.setStartingPose(scorePose);
        } else if (gamepad1.right_bumper) {
            BlueAlliance = false;
            follower.setStartingPose(red_Score);
        }
    }

    // Called once when we hit PLAY
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        changePath(0);
    }

    // LOOPS as the op mode is running
    @Override
    public void loop() {
        follower.update();
        PathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("time left", 30 - opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();

        if (opmodeTimer.getElapsedTimeSeconds() > 30.0) {
            return;
        }
    }
}

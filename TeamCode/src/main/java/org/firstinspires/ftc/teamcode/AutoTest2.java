package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Decode Auto", group = "Autonomous")
public class AutoTest2 extends OpMode {

    private Follower follower;

    private DcMotor rotate, intake, shooter;
    private Servo blocker, pusher, hood;

    private boolean BlueAlliance = true;

    /* ---------------- AUTO STATE MACHINE ---------------- */

    private enum AutoState {
        LOAD,
        PICKUP1,
        PICKUP2,
        PICKUP3,
        IDLE,
        SHOOTING
    }

    private AutoState state, laststate;

    /* ---------------- FIELD POSES ---------------- */

    private final Pose scorePose = new Pose(64, 80, Math.toRadians(140));
    private final Pose loadPose = new Pose(10, 9, Math.toRadians(180));

    private final Pose pickup1Pose = new Pose(23, 84.85, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(23, 60, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(23, 35, Math.toRadians(180));

    private final Pose redScorePose = new Pose(80, 80, Math.toRadians(40));
    private final Pose redLoadPose = new Pose(140, 5, Math.toRadians(0));

    private final Pose redPickup1Pose = new Pose(121, 84.85, Math.toRadians(0));
    private final Pose redPickup2Pose = new Pose(121, 60, Math.toRadians(0));
    private final Pose redPickup3Pose = new Pose(121, 35, Math.toRadians(0));

    /* ---------------- PATHS ---------------- */

    private PathChain loadPath, pickupPath1, pickupPath2, pickupPath3;
    private PathChain redLoadPath, redPickupPath1, redPickupPath2, redPickupPath3;

    /* ---------------- SHOOT CALLBACK ---------------- */

    Runnable shootShooter = () -> {
        laststate = state
        state = AutoState.SHOOTING;
    };

    /* ---------------- CREATE PATHS ---------------- */

    private void createPaths() {

        loadPath = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(72, 30), loadPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), loadPose.getHeading())
                .addParametricCallback(1.0, () -> intake.setPower(-0.8))

                .addPath(new BezierCurve(loadPose, new Pose(72, 30), scorePose))
                .setLinearHeadingInterpolation(loadPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.2, () -> intake.setPower(0))
                .addParametricCallback(1.0, shootShooter)
                .build();

        redLoadPath = follower.pathBuilder()
                .addPath(new BezierCurve(redScorePose, new Pose(72, 30), redLoadPose))
                .setLinearHeadingInterpolation(redScorePose.getHeading(), redLoadPose.getHeading())
                .addParametricCallback(1.0, () -> intake.setPower(-0.8))

                .addPath(new BezierCurve(redLoadPose, new Pose(72, 30), redScorePose))
                .setLinearHeadingInterpolation(redLoadPose.getHeading(), redScorePose.getHeading())
                .addParametricCallback(0.2, () -> intake.setPower(0))
                .addParametricCallback(1.0, shootShooter)
                .build();

        pickupPath1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addParametricCallback(0.3, () -> intake.setPower(-0.8))
                .addParametricCallback(1.0, () -> intake.setPower(0))

                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(1.0, shootShooter)
                .build();

        pickupPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(65, 52), pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addParametricCallback(0.3, () -> intake.setPower(-0.8))
                .addParametricCallback(1.0, () -> intake.setPower(0))

                .addPath(new BezierCurve(pickup2Pose, new Pose(65, 52), scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(1.0, shootShooter)
                .build();

        pickupPath3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(75, 30), pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .addParametricCallback(0.3, () -> intake.setPower(-0.8))
                .addParametricCallback(1.0, () -> intake.setPower(0))

                .addPath(new BezierCurve(pickup3Pose, new Pose(75, 30), scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(1.0, shootShooter)
                .build();

        redPickupPath1 = follower.pathBuilder()
                .addPath(new BezierLine(redScorePose, redPickup1Pose))
                .setLinearHeadingInterpolation(redScorePose.getHeading(), redPickup1Pose.getHeading())
                .addParametricCallback(0.3, () -> intake.setPower(-0.8))
                .addParametricCallback(1.0, () -> intake.setPower(0))

                .addPath(new BezierLine(redPickup1Pose, redScorePose))
                .setLinearHeadingInterpolation(redPickup1Pose.getHeading(), redScorePose.getHeading())
                .addParametricCallback(1.0, shootShooter)
                .build();

        redPickupPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(redScorePose, new Pose(79, 52), redPickup2Pose))
                .setLinearHeadingInterpolation(redScorePose.getHeading(), redPickup2Pose.getHeading())
                .addParametricCallback(0.3, () -> intake.setPower(-0.8))
                .addParametricCallback(1.0, () -> intake.setPower(0))

                .addPath(new BezierCurve(redPickup2Pose, new Pose(79, 52), redScorePose))
                .setLinearHeadingInterpolation(redPickup2Pose.getHeading(), redScorePose.getHeading())
                .addParametricCallback(1.0, shootShooter)
                .build();

        redPickupPath3 = follower.pathBuilder()
                .addPath(new BezierCurve(redScorePose, new Pose(69, 30), redPickup3Pose))
                .setLinearHeadingInterpolation(redScorePose.getHeading(), redPickup3Pose.getHeading())
                .addParametricCallback(0.3, () -> intake.setPower(-0.8))
                .addParametricCallback(1.0, () -> intake.setPower(0))

                .addPath(new BezierCurve(redPickup3Pose, new Pose(69, 30), redScorePose))
                .setLinearHeadingInterpolation(redPickup3Pose.getHeading(), redScorePose.getHeading())
                .addParametricCallback(1.0, shootShooter)
                .build();
    }

    /* ---------------- MECHANISMS ---------------- */

    private void defineMechanisms() {
        rotate = hardwareMap.get(DcMotor.class, "rotate");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        hood = hardwareMap.get(Servo.class, "hood");
        blocker = hardwareMap.get(Servo.class, "blocker");
        pusher = hardwareMap.get(Servo.class, "pusher");

        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /* ---------------- STATE MACHINE ---------------- */

    private void autoUpdate() {

        switch (state) {

            case LOAD:

                if (!follower.isBusy()) {
                    follower.followPath(BlueAlliance ? loadPath : redLoadPath);
                    state = AutoState.PICKUP1;
                }

                break;

            case PICKUP1:

                if (!follower.isBusy()) {
                    follower.followPath(BlueAlliance ? pickupPath1 : redPickupPath1);
                    state = AutoState.PICKUP2;
                }

                break;

            case PICKUP2:

                if (!follower.isBusy()) {
                    follower.followPath(BlueAlliance ? pickupPath2 : redPickupPath2);
                    state = AutoState.PICKUP3;
                }

                break;

            case PICKUP3:

                if (!follower.isBusy()) {
                    follower.followPath(BlueAlliance ? pickupPath3 : redPickupPath3);
                    state = AutoState.IDLE;
                }

                break;

            case SHOOTING:
                break;
            case IDLE:
                break;
        }
    }

    /* ---------------- OPMODE METHODS ---------------- */

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        defineMechanisms();
        createPaths();

        state = AutoState.LOAD;
    }

    @Override
    public void init_loop() {

        if (gamepad1.left_bumper) {
            BlueAlliance = true;
            follower.setStartingPose(scorePose);
        }

        if (gamepad1.right_bumper) {
            BlueAlliance = false;
            follower.setStartingPose(redScorePose);
        }
    }

    @Override
    public void loop() {

        follower.update();
        autoUpdate();

        telemetry.addData("State", state);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());

        telemetry.update();
    }
}
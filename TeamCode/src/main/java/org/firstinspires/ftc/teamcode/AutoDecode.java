package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * AutoDecode — FTC 2025-26 Autonomous
 * Strategy (in priority order):
 *   1. Pick up field balls (3 rows)
 *   2. Shoot into goal
 *   3. Load zone pickup
 *   4. Park (implicit — robot stays near goal after final shot)
 * Alliance is selected and paths and poses finalised during init_loop via gamepad bumpers.
 * All hardware names match FinalRobotCode.java and Constants.java.
 * State machine overview:
 *   SHOOT_5 -> LOAD and SHOOT -> ROW 1 and SHOOT -> ROW 2 and SHOOT -> ROW 3 and SHOOT ->
 *   DONE.
 * Each state waits for the current path to finish (with a timeout)
 * before activating mechanisms and transitioning to the next state.
 */

@Autonomous(name = "Decode Auto 2025-26", group = "Autonomous")
public class AutoDecode extends OpMode {

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
    private Follower follower;
    private DcMotor  intake;
    private DcMotor  shooter;

    // -------------------------------------------------------------------------
    // Timers
    // -------------------------------------------------------------------------
    private Timer stateTimer;   // how long we've been in the current state
    private Timer matchTimer;   // total elapsed autonomous time

    // -------------------------------------------------------------------------
    // Timeouts (seconds) — adjust to taste
    // -------------------------------------------------------------------------
    private static final double PATH_TIMEOUT_S    = 5.0;  // give up waiting for path after this
    private static final double INTAKE_TIME_S     = 1.5;  // run intake while stopped at row
    private static final double SHOOTER_SPINUP_S  = 1.2;  // spin up before pushing ball
    private static final double SHOOT_TIME_S      = 0.8;  // time to actually fire
    private static final double MATCH_DURATION_S  = 30.0; // standard auto period

    private static final double SHOOTER_POWER =  1.0;
    private static final double INTAKE_IN     = -0.8;  // pull balls in
    private static final double INTAKE_OFF    =  0.0;

    private boolean blueAlliance = true;
    private enum State {
        IDLE,
        DRIVE_TO_ROW_1,
        INTAKE_ROW_1,
        DRIVE_TO_SHOOT_1,
        SHOOT_1,
        DRIVE_TO_ROW_2,
        INTAKE_ROW_2,
        DRIVE_TO_SHOOT_2,
        SHOOT_2,
        DRIVE_TO_ROW_3,
        INTAKE_ROW_3,
        DRIVE_TO_SHOOT_3,
        SHOOT_3,
        DRIVE_TO_LOAD,
        INTAKE_LOAD,
        DRIVE_TO_SHOOT_4,
        SHOOT_4,
        SHOOT_5,
        DONE
    }
    private State state = State.IDLE;

    // TODO check and confirm poses
    // Blue
    private final Pose BLUE_SCORE   = new Pose( 64,  80, Math.toRadians(140));
    private final Pose BLUE_ROW_1   = new Pose( 23,  85, Math.toRadians(180));
    private final Pose BLUE_ROW_2   = new Pose( 23,  60, Math.toRadians(180));
    private final Pose BLUE_ROW_3   = new Pose( 23,  35, Math.toRadians(180));
    private final Pose BLUE_LOAD    = new Pose( 10,   9, Math.toRadians(180));

    // Red (Y-mirrored)
    private final Pose RED_SCORE    = new Pose( 80,  80, Math.toRadians( 40));
    private final Pose RED_ROW_1    = new Pose(121,  85, Math.toRadians(  0));
    private final Pose RED_ROW_2    = new Pose(121,  60, Math.toRadians(  0));
    private final Pose RED_ROW_3    = new Pose(121,  35, Math.toRadians(  0));
    private final Pose RED_LOAD     = new Pose(140,   5, Math.toRadians(  0));

    //Active poses
    private Pose pScore, pRow1, pRow2, pRow3, pLoad;
    private PathChain pathToRow1, pathToShoot1;
    private PathChain pathToRow2, pathToShoot2;
    private PathChain pathToRow3, pathToShoot3;
    private PathChain pathToLoad, pathToShoot4;

    @Override
    public void init() {
        // Follower
        follower = Constants.createFollower(hardwareMap);

        // Mechanisms
        intake  = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // flywheels coast

        // Timers
        stateTimer = new Timer();
        matchTimer = new Timer();

        telemetry.addLine("Initialised — waiting for alliance selection.");
        telemetry.addLine("Left bumper = BLUE   |   Right bumper = RED");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Alliance selection
        if (gamepad1.left_bumper) {
            blueAlliance = true;
        } else if (gamepad1.right_bumper) {
            blueAlliance = false;
        }

        if (blueAlliance) {
            pScore = BLUE_SCORE;
            pRow1  = BLUE_ROW_1;
            pRow2  = BLUE_ROW_2;
            pRow3  = BLUE_ROW_3;
            pLoad  = BLUE_LOAD;
        } else {
            pScore = RED_SCORE;
            pRow1  = RED_ROW_1;
            pRow2  = RED_ROW_2;
            pRow3  = RED_ROW_3;
            pLoad  = RED_LOAD;
        }

        follower.setStartingPose(pScore);
        buildPaths();
        // Update starting pose for field drawing
        follower.setStartingPose(blueAlliance ? BLUE_SCORE : RED_SCORE);

        telemetry.addData("Alliance", blueAlliance ? ">>> BLUE <<<" : ">>> RED  <<<");
        telemetry.addLine("Press PLAY when ready.");
        telemetry.update();
    }

    @Override
    public void start() {
        matchTimer.resetTimer();
        follower.followPath(pathToRow1);
        shooter.setPower(SHOOTER_POWER);
        transitionTo(State.SHOOT_5); // TODO check starting state
    }

    @Override
    public void loop() {
        follower.update();

        // Hard stop if match time is almost up
        if (matchTimer.getElapsedTimeSeconds() >= MATCH_DURATION_S) {
            stopAllMechanisms();
            state = State.DONE;
        }

        switch (state) {

            // -----------------------------------------------------------------
            case IDLE:
                // Waiting — nothing to do
                break;

            // -----------------------------------------------------------------
            // ROW 1
            // -----------------------------------------------------------------
            case DRIVE_TO_ROW_1:
                if (pathFinishedOrTimedOut()) {
                    intake.setPower(INTAKE_IN);
                    transitionTo(State.INTAKE_ROW_1);
                }
                break;

            case INTAKE_ROW_1:
                if (stateTimer.getElapsedTimeSeconds() >= INTAKE_TIME_S) {
                    intake.setPower(INTAKE_OFF);
                    follower.followPath(pathToShoot1, true);
                    transitionTo(State.DRIVE_TO_SHOOT_1);
                }
                break;

            case DRIVE_TO_SHOOT_1:
                if (pathFinishedOrTimedOut()) {
                    shooter.setPower(SHOOTER_POWER);
                    transitionTo(State.SHOOT_1);
                }
                break;

            case SHOOT_1:
                if (stateTimer.getElapsedTimeSeconds() >= SHOOTER_SPINUP_S + SHOOT_TIME_S) {
                    shooter.setPower(0);
                    follower.followPath(pathToRow2, true);
                    transitionTo(State.DRIVE_TO_ROW_2); //TODO check
                }
                break;

            // -----------------------------------------------------------------
            // ROW 2
            // -----------------------------------------------------------------
            case DRIVE_TO_ROW_2:
                if (pathFinishedOrTimedOut()) {
                    intake.setPower(INTAKE_IN);
                    transitionTo(State.INTAKE_ROW_2);
                }
                break;

            case INTAKE_ROW_2:
                if (stateTimer.getElapsedTimeSeconds() >= INTAKE_TIME_S) {
                    intake.setPower(INTAKE_OFF);
                    follower.followPath(pathToShoot2, true);
                    transitionTo(State.DRIVE_TO_SHOOT_2);
                }
                break;

            case DRIVE_TO_SHOOT_2:
                if (pathFinishedOrTimedOut()) {
                    shooter.setPower(SHOOTER_POWER);
                    transitionTo(State.SHOOT_2);
                }
                break;

            case SHOOT_2:
                if (stateTimer.getElapsedTimeSeconds() >= SHOOTER_SPINUP_S + SHOOT_TIME_S) {
                    shooter.setPower(0);
                    follower.followPath(pathToRow3, true);
                    transitionTo(State.DRIVE_TO_ROW_3); //TODO check
                }
                break;

            // -----------------------------------------------------------------
            // ROW 3
            // -----------------------------------------------------------------
            case DRIVE_TO_ROW_3:
                if (pathFinishedOrTimedOut()) {
                    intake.setPower(INTAKE_IN);
                    transitionTo(State.INTAKE_ROW_3);
                }
                break;

            case INTAKE_ROW_3:
                if (stateTimer.getElapsedTimeSeconds() >= INTAKE_TIME_S) {
                    intake.setPower(INTAKE_OFF);
                    follower.followPath(pathToShoot3, true);
                    transitionTo(State.DRIVE_TO_SHOOT_3);
                }
                break;

            case DRIVE_TO_SHOOT_3:
                if (pathFinishedOrTimedOut()) {
                    shooter.setPower(SHOOTER_POWER);
                    transitionTo(State.SHOOT_3);
                }
                break;

            case SHOOT_3:
                if (stateTimer.getElapsedTimeSeconds() >= SHOOTER_SPINUP_S + SHOOT_TIME_S) {
                    shooter.setPower(0);
                    follower.followPath(pathToLoad, true);
                    transitionTo(State.DRIVE_TO_LOAD); //TODO check
                }
                break;

            // -----------------------------------------------------------------
            // LOAD ZONE
            // -----------------------------------------------------------------
            case DRIVE_TO_LOAD:
                if (pathFinishedOrTimedOut()) {
                    intake.setPower(INTAKE_IN);
                    transitionTo(State.INTAKE_LOAD);
                }
                break;

            case INTAKE_LOAD:
                if (stateTimer.getElapsedTimeSeconds() >= INTAKE_TIME_S) {
                    intake.setPower(INTAKE_OFF);
                    follower.followPath(pathToShoot4, true);
                    transitionTo(State.DRIVE_TO_SHOOT_4);
                }
                break;

            case DRIVE_TO_SHOOT_4:
                if (pathFinishedOrTimedOut()) {
                    shooter.setPower(SHOOTER_POWER);
                    transitionTo(State.SHOOT_4);
                }
                break;

            case SHOOT_4:
                if (stateTimer.getElapsedTimeSeconds() >= SHOOTER_SPINUP_S + SHOOT_TIME_S) {
                    stopAllMechanisms();
                    transitionTo(State.DRIVE_TO_ROW_1);
                }
                break;

            case SHOOT_5:
                if (stateTimer.getElapsedTimeSeconds() >= SHOOTER_SPINUP_S + SHOOT_TIME_S) {
                    stopAllMechanisms();
                    transitionTo(State.DRIVE_TO_LOAD);
                }
                break;
            // -----------------------------------------------------------------
            case DONE:
                stopAllMechanisms();
                break;
        }

        // Telemetry
        telemetry.addData("Alliance",    blueAlliance ? "BLUE" : "RED");
        telemetry.addData("State",       state);
        telemetry.addData("State time",  "%.1f s", stateTimer.getElapsedTimeSeconds());
        telemetry.addData("Match time",  "%.1f s", matchTimer.getElapsedTimeSeconds());
        telemetry.addData("Follower busy", follower.isBusy());
        telemetry.addData("X",  "%.1f", follower.getPose().getX());
        telemetry.addData("Y",  "%.1f", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }


    // Helper methods
    private void transitionTo(State next) {
        state = next;
        stateTimer.resetTimer();
    }
    private boolean pathFinishedOrTimedOut() {
        return !follower.isBusy()
                || stateTimer.getElapsedTimeSeconds() >= PATH_TIMEOUT_S;
    }
    private void stopAllMechanisms() {
        intake.setPower(0);
        shooter.setPower(0);
    }
    private void buildPaths() {

        // Score → Row 1 (straight line, row 1 is near the goal side)
        pathToRow1 = follower.pathBuilder()
                .addPath(new BezierLine(pScore, pRow1))
                .setLinearHeadingInterpolation(pScore.getHeading(), pRow1.getHeading())
                .build();

        // Row 1 → Score
        pathToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(pRow1, pScore))
                .setLinearHeadingInterpolation(pRow1.getHeading(), pScore.getHeading())
                .build();

        // Score → Row 2 (slight curve to avoid field elements)
        Pose midScore2 = blueAlliance
                ? new Pose(65, 52, pScore.getHeading())
                : new Pose(79, 52, pScore.getHeading());

        pathToRow2 = follower.pathBuilder()
                .addPath(new BezierCurve(pScore, midScore2, pRow2))
                .setLinearHeadingInterpolation(pScore.getHeading(), pRow2.getHeading())
                .build();

        pathToShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(pRow2, midScore2, pScore))
                .setLinearHeadingInterpolation(pRow2.getHeading(), pScore.getHeading())
                .build();

        // Score → Row 3 (curve around centre)
        Pose midScore3 = blueAlliance
                ? new Pose(75, 30, pScore.getHeading())
                : new Pose(69, 30, pScore.getHeading());

        pathToRow3 = follower.pathBuilder()
                .addPath(new BezierCurve(pScore, midScore3, pRow3))
                .setLinearHeadingInterpolation(pScore.getHeading(), pRow3.getHeading())
                .build();

        pathToShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(pRow3, midScore3, pScore))
                .setLinearHeadingInterpolation(pRow3.getHeading(), pScore.getHeading())
                .build();

        // Score → Load zone (wide curve to stay clear of field balls)
        Pose midLoad = new Pose(72, 30, pScore.getHeading());

        pathToLoad = follower.pathBuilder()
                .addPath(new BezierCurve(pScore, midLoad, pLoad))
                .setLinearHeadingInterpolation(pScore.getHeading(), pLoad.getHeading())
                .build();

        pathToShoot4 = follower.pathBuilder()
                .addPath(new BezierCurve(pLoad, midLoad, pScore))
                .setLinearHeadingInterpolation(pLoad.getHeading(), pScore.getHeading())
                .build();
    } //TODO finish and confirm paths
}
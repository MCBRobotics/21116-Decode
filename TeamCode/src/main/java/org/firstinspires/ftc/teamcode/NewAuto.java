package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "ExperimentalAuto", group = "Autonomous")
public class NewAuto extends OpMode {
    private Follower follower;

    //***** DECLARE POSES AND PATHS *****//
    PathChain firstLine, firstCurve;
    Pose startPose = new Pose(72, 72, Math.toRadians(90));
    Pose awayPose = new Pose(72, 100, Math.toRadians(180));
    private void buildPaths() {
        firstLine = follower.pathBuilder()
                .addPath(new BezierLine(startPose, awayPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), awayPose.getHeading())
                .addPath(new BezierLine(awayPose, startPose))
                .setLinearHeadingInterpolation(awayPose.getHeading(), startPose.getHeading())
                .build();

        firstCurve = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, new Pose(63,83), awayPose))
                .build();
    }

    //***** OP_MODE METHODS *****//
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        setCurrentPathState(PathState.FIRST_LINE);
        buildPaths();
    }

    @Override
    public void loop() {
        switch (currentPathState) {
            case IDLE:
                break;
            case FIRST_LINE:
                if (!follower.isBusy()) {
                    follower.followPath(firstLine);
                    follower.breakFollowing();
                    setCurrentPathState(PathState.FIRST_CURVE);
                }
                break;
            case FIRST_CURVE:
                if (!follower.isBusy()) {
                    follower.followPath(firstCurve);
                    follower.breakFollowing();
                    setCurrentPathState(PathState.IDLE);
                }
                break;
        }

    }

    //***** STATE MACHINE *****//
    enum PathState {
        IDLE,
        FIRST_LINE,
        FIRST_CURVE
    }
    PathState currentPathState = null;
    private void setCurrentPathState(PathState pathState) {currentPathState = pathState;}

}
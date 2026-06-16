package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


public class NewAuto extends OpMode {

    private Follower follower;

    Path forwardLine = new Path(
            new BezierLine(
                    new Pose(20.0, 100, Math.toRadians(90)),
                    new Pose(20.0, 72, Math.toRadians(180))
            )
    );


    PathChain firstLine1 = new PathChain(
            forwardLine,
            new Path(new BezierLine(
                    follower.getPose(),
                    new Pose(20.0, 100, Math.toRadians(90))
            ))
    );

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0.0, 0.0, Math.toRadians(0)));
    }

    @Override
    public void loop() {
        follower.followPath(firstLine1);
    }
}
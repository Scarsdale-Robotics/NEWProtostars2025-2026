package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class CloseLeftBackupAuto extends LinearOpMode {
    public Follower follower;
    public Timer pathTimer;
    public Timer opModeTimer;
    public RobotSystem robot;
    public int pathState = 0;
    public Pose startPose = new Pose(60,10,90);
    public Pose outside = new Pose(60,40,90);
    public PathChain parkAndReturn;
    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        this.pathTimer = new Timer();
        this.opModeTimer = new Timer();
        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
        buildPaths();
    }
    public void buildPaths() {
        this.parkAndReturn = follower.pathBuilder()
                .setGlobalConstantHeadingInterpolation(90)
                .addPath(new BezierLine(startPose, outside))
                .addPath(new BezierLine(outside,startPose))
                .build();
    }
    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                if (!(follower.isBusy())) {
                    follower.followPath(parkAndReturn);
                }
                break;
            case 1:
                setPathState(-1);
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}

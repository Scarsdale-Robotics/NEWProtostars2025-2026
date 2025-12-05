package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//TODO: tune drive constants for getting to pos
//TODO: experiment with holdend
@Autonomous(name = "FINALFLAUTO")
public class FarLeftBackupAuto extends LinearOpMode {
    public Follower follower;
    public Timer pathTimer;
    public Timer opModeTimer;
    public RobotSystem robot;
    public int pathState = 0;
    public Pose startPose = new Pose(40,134,Math.toRadians(270));
    public Pose outside = new Pose(50,70,Math.toRadians(270));
    public PathChain parkAndReturn;
    public PathChain returnPath;
    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        robot.inDep.initControllers();
        this.pathTimer = new Timer();
        this.opModeTimer = new Timer();
        buildPaths();
        setPathState(0);
        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
    public void buildPaths() {
        this.parkAndReturn = follower.pathBuilder()
                .addPath(new BezierLine(startPose, outside))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();
        this.returnPath = follower.pathBuilder()
                .addPath(new BezierLine(outside, startPose))
                .setGlobalConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(parkAndReturn);
                if (follower.atPose(outside, 1, 1)) {
                    setPathState(1);
                }
                break;
            case 1:
                follower.followPath(returnPath);
                if (follower.atPose(startPose, 2,2)) {
                    setPathState(2);
                }
                break;
            case 2:
                setPathState(-1);
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}

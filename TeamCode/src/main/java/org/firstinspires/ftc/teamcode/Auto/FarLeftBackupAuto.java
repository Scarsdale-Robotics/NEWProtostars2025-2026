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
//TODO: Change everything to radians'
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
        robot.hardwareRobot.initOdom();
        robot.inDep.initControllers();
        this.pathTimer = new Timer();
        this.opModeTimer = new Timer();
        buildPaths();
        setPathState(0);
        waitForStart();
        while (opModeIsActive()) {
            robot.hardwareRobot.pinpoint.update();
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
                .setGlobalConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(new BezierLine(startPose, outside))
                .build();
        this.returnPath = follower.pathBuilder()
                .addPath(new BezierLine(outside, startPose))
                .setGlobalConstantHeadingInterpolation(Math.toRadians(270))
                .build();
    }
    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                if (!(follower.isBusy())) {
                    follower.followPath(parkAndReturn);
                    setPathState(1);
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(returnPath);
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

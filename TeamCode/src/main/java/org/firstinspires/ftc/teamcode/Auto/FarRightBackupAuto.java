package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//TODO: tune drive constants for getting to pos
//TODO: experiment with holdend
@Autonomous (name = "FINALFRAUTO")
public class FarRightBackupAuto extends LinearOpMode {
    public Follower follower;
    public Timer pathTimer;
    public Timer opModeTimer;
    public RobotSystem robot;
    public int pathState = 0;
    public Pose startPose = new Pose(105,134,Math.toRadians(270));
    public Pose outside = new Pose(90,70,Math.toRadians(270));
    public PathChain parkAndReturn;
    public PathChain returnPath;
    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        this.pathTimer = new Timer();
        this.opModeTimer = new Timer();
        opModeTimer.resetTimer();
        robot.inDep.initControllers();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        setPathState(0);
        follower.setStartingPose(startPose);
        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("Follower Pose", follower.getPose());
            telemetry.addData("StartPose X", startPose.getX());
            telemetry.addData("StartPose Y", startPose.getY());
            telemetry.addData("StartPose Heading", startPose.getHeading());
            telemetry.addData("Outside X", outside.getX());
            telemetry.addData("Outside Y", outside.getY());
            telemetry.addData("Outside Heading", outside.getHeading());
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
                if (follower.atPose(outside, 2, 2)) {
                    setPathState(1);
                }
                break;
            case 1:
                follower.followPath(returnPath);
                if (follower.atPose(startPose,2,2)) {
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

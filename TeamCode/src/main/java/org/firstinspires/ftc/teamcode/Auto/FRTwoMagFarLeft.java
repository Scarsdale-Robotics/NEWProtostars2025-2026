package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

//This does not work.
@Autonomous(name = "FR2MagFLeft")
public class FRTwoMagFarLeft extends LinearOpMode {
    public RobotSystem robot;
    public PathChain scorePreloadPath;
    public PathChain pickupOnePath;
    public PathChain finishPickupOnePath;
    public PathChain scorePickupOnePath;
    public PathChain pickupTwoPath;
    public PathChain finishPickupTwoPath;
    public PathChain scorePickupTwoPath;
    public PathChain returnPath;
    public Follower follower;
    public Timer pathTimer, opmodeTimer;
    public int pathState;
    public Pose startPose = new Pose(37,134,Math.toRadians(270));
    public Pose pickupOne = new Pose(41, 84, Math.toRadians(180));
    public Pose pickupOneFinish = new Pose(29, 81, Math.toRadians(180));
    public Pose alignGoal = new Pose(67, 90, Math.toRadians(143));
    public Pose pickupTwo = new Pose(41,60,Math.toRadians(180));
    public Pose finishPickupTwo = new Pose(29,60,Math.toRadians(180));
    public AprilTagDetection lastTagDetected;
    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        setPathState(-1);
        follower.setStartingPose(startPose);
        buildPaths();
        robot.inDep.setShooterPower(1);
        waitForStart();
        while (opModeIsActive()) {
            detectTags();
            follower.update();
            autonomousPathUpdate();
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
    public boolean xInchRadius(int radius, AprilTagDetection target) {
        return target.ftcPose.range <= radius;
    }
    public void buildPaths() {
        this.scorePreloadPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, alignGoal))
                .setLinearHeadingInterpolation(startPose.getHeading(), alignGoal.getHeading())
                .build();
        this.pickupOnePath = follower.pathBuilder()
                .addPath(new BezierLine(alignGoal, pickupOne))
                .setLinearHeadingInterpolation(alignGoal.getHeading(), pickupOne.getHeading())
                .build();
        this.finishPickupOnePath = follower.pathBuilder()
                .addPath(new BezierLine(pickupOne, pickupOneFinish))
                .setConstantHeadingInterpolation(pickupOne.getHeading())
                .build();
        this.scorePickupOnePath = follower.pathBuilder()
                .addPath(new BezierLine(pickupOneFinish, alignGoal))
                .setLinearHeadingInterpolation(pickupOneFinish.getHeading(), alignGoal.getHeading())
                .build();
        this.pickupTwoPath = follower.pathBuilder()
                .addPath(new BezierLine(alignGoal, pickupTwo))
                .setLinearHeadingInterpolation(alignGoal.getHeading(), pickupTwo.getHeading())
                .build();
        this.finishPickupTwoPath = follower.pathBuilder()
                .addPath(new BezierLine(pickupTwo, finishPickupTwo))
                .setConstantHeadingInterpolation(pickupTwo.getHeading())
                .build();
        this.scorePickupTwoPath = follower.pathBuilder()
                .addPath(new BezierLine(finishPickupTwo, alignGoal))
                .setLinearHeadingInterpolation(finishPickupTwo.getHeading(), alignGoal.getHeading())
                .build();
        this.returnPath = follower.pathBuilder()
                .addPath(new BezierLine(alignGoal, startPose))
                .setLinearHeadingInterpolation(alignGoal.getHeading(), startPose.getHeading())
                .build();
    }
    public void detectTags() {
        ArrayList<AprilTagDetection> detections = robot.cv.aprilTagProcessor.getDetections();
        if (detections != null && !detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                telemetry.addLine("AprilTag Detected.");
                telemetry.addData("ID", tag.id);
                telemetry.addData("X (Sideways offset)", tag.ftcPose.x);
                telemetry.addData("Y (Forward/Back Offset)", tag.ftcPose.y);
                telemetry.addData("Z", tag.ftcPose.z);
                telemetry.addData("Bearing", tag.ftcPose.bearing);
                telemetry.addData("Yaw", tag.ftcPose.yaw);
                telemetry.addData("Range: ", tag.ftcPose.range);
                lastTagDetected = tag;
                break;
            }
        } else {
            lastTagDetected = null; // clear old tag when none detected
        }
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) follower.followPath(scorePreloadPath);
                robot.inDep.unloadMag(opmodeTimer);
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(pickupOnePath);
                    robot.inDep.setIntake(0.6);
                }
                setPathState(2);
                break;
            case 2:
                if (!follower.isBusy()) follower.followPath(finishPickupOnePath);
                robot.inDep.setIntake(0);
                setPathState(3);
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickupOnePath);
                }
                robot.inDep.unloadMag(opmodeTimer);
                setPathState(4);
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(pickupTwoPath);
                }
                robot.inDep.setIntake(0.6);
                setPathState(5);
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(finishPickupTwoPath);
                }
                robot.inDep.setIntake(0);
                setPathState(6);
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickupTwoPath);
                }
                robot.inDep.unloadMag(opmodeTimer);
                setPathState(7);
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(returnPath);
                }
                setPathState(8);
                break;
            case 8:
                if (!follower.isBusy()) setPathState(-1);
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    private Pose getRobotPoseFromCamera(AprilTagDetection tag) {
        return new Pose(tag.robotPose.getPosition().x, tag.robotPose.getPosition().y, follower.getHeading(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
    public AprilTagDetection getTargetTag(int targetId1, int targetId2, int targetId3) {
        List<AprilTagDetection> detections = robot.cv.aprilTagProcessor.getDetections();
        AprilTagDetection sol = null;
        for (AprilTagDetection tag : detections) {
            if (tag.id == targetId1 || tag.id == targetId2 || tag.id == targetId3) {
                sol = tag;
                break;
            }
        }
        return sol;
    }
}

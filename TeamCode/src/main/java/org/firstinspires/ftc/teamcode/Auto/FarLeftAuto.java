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

//TODO: TUNE PID, CENTRIPETAL, ALL CONSTANTS, AND EXPERIMENT WITH INTERPOLATION.
//TODO: download ftcdashboard and tune constants with drive test
//TODO: Check poses for accurate data on visualizer
//TODO: fix pickup sequence for one big sweep
@Autonomous(name = "FarLeftAuto")
public class FarLeftAuto extends LinearOpMode {
    public RobotSystem robot = new RobotSystem(hardwareMap, this);
    public PathChain detectionPathChain;
    public PathChain returnPathChain;
    public PathChain pickupPathChain1;
    public PathChain pickupPathChain2;
    public PathChain intakePathChain;
    public PathChain pickupPathChain3;
    public PathChain scorePathChain;
    public PathChain scorePreloadPathChain;
    public Follower follower;
    public Timer pathTimer, opmodeTimer;
    public int pathState;
    public final Pose startPose = new Pose(40,134,Math.toRadians(270));
    public final Pose apTag1 = new Pose(70,80, Math.toRadians(90));
    //quad bezier curve, rest linear w little to no interpolation
    public Pose pickup = new Pose(43, 81, Math.toRadians(180));
    // quadratic bezier curve for this step
    public Pose alignGoal = new Pose(40, 115, Math.toRadians(143));
    public PathChain finishIntake;
    public PathChain scorePreloadPathChainPtTwo;
    public AprilTagDetection lastTagDetected;
    @Override
    public void runOpMode() throws InterruptedException {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        setPathState(-1);
        follower.setStartingPose(startPose);
        robot.hardwareRobot.initOdom();
        buildPaths();
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
        this.scorePreloadPathChain = follower.pathBuilder()
                .addPath(new BezierLine(apTag1, alignGoal))
                .setLinearHeadingInterpolation(Math.toRadians(90),Math.toRadians(143))
                .build();
        this.scorePreloadPathChainPtTwo = follower.pathBuilder()
                .addPath(new BezierLine(alignGoal, apTag1))
                .setLinearHeadingInterpolation(Math.toRadians(143),Math.toRadians(90))
                .build();
        this.detectionPathChain = follower.pathBuilder()
                .addPath(new BezierLine(startPose, apTag1))
                .setLinearHeadingInterpolation(Math.toRadians(270),Math.toRadians(90))
                .build();
        this.pickupPathChain1 = follower.pathBuilder()
                .addPath(new BezierLine(apTag1,pickup))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
        this.pickupPathChain2 = follower.pathBuilder()
                .addPath(new BezierLine(apTag1, new Pose(43, 56,0)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
        this.pickupPathChain3 = follower.pathBuilder()
                .addPath(new BezierLine(apTag1, new Pose(43, 30, Math.toRadians(0))))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
        this.scorePathChain = follower.pathBuilder()
                .addPath(new BezierCurve(Arrays.asList(follower.getPose(), new Pose(70,80, Math.toRadians(0)), alignGoal)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                .build();
        this.returnPathChain = follower.pathBuilder()
                .addPath(new BezierLine(alignGoal,startPose))
                .setLinearHeadingInterpolation(Math.toRadians(143),Math.toRadians(270))
                .build();
        this.intakePathChain = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), new Pose(follower.getPose().getX() - 6, follower.getPose().getY(), follower.getHeading())))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        this.finishIntake = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), new Pose(follower.getPose().getX() + 18, follower.getPose().getY(), follower.getHeading())))
                .setConstantHeadingInterpolation(Math.toRadians(180))
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
                if (!follower.isBusy()) follower.followPath(scorePreloadPathChain);
                //shoot
                if (!follower.isBusy()) {
                    follower.followPath(scorePreloadPathChainPtTwo);
                    setPathState(1);
                }
                break;
            case -1:
                if (!follower.isBusy())  {
                    follower.followPath(detectionPathChain);
                    setPathState(0);
                }
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    Pose current = follower.getPose();
                    Pose vision = getRobotPoseFromCamera(getTargetTag(24,24,24));
                    Pose blended = new Pose(
                            (current.getX() * 0.8 + vision.getX() * 0.2),
                            (current.getY() * 0.8 + vision.getY() * 0.2),
                            vision.getHeading()
                    );
                    follower.setPose(blended);
                    if (robot.decode(getTargetTag(21,22,23)).equals("PPG")) follower.followPath(pickupPathChain3);
                    else if (robot.decode(getTargetTag(21,22,23)).equals("PGP")) follower.followPath(pickupPathChain2);
                    else follower.followPath(pickupPathChain1);
                    follower.followPath(intakePathChain);
                    //intake
                    follower.followPath(intakePathChain);
                    //intake
                    follower.followPath(intakePathChain);
                    //intake
                    follower.followPath(finishIntake);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(scorePathChain);
                }
                if (!follower.isBusy()) {
                    follower.followPath(returnPathChain);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    setPathState(-2);
                }
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

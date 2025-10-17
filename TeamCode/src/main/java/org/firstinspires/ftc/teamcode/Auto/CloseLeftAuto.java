package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.ArrayList;
import java.util.Arrays;
// not finished
//TODO: TUNE PID, CENTRIPETAL, ALL CONSTANTS, AND EXPERIMENT WITH INTERPOLATION.
//TODO: download ftcdashboard and tune constants with drive test
//TODO: finish path implementation: fix preload path
//TODO: make 4 other autos, make backup autos, and add intake steps to pathing.
//TODO: add move to load zone for pathing
//TODO: Change everything to radians
//TODO: edit poses so that they actually align and clean up code
public class CloseLeftAuto extends LinearOpMode {
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
    public final Pose startPose = new Pose(60,10,90);
    public final Pose apTag1 = new Pose(70,80, startPose.getHeading());
    //quad bezier curve, rest linear w little to no interpolation
    public Pose pickup = new Pose(40, 82, 180);
    // quadratic bezier curve for this step
    public Pose alignGoal = new Pose(40, 115, 143);
    public PathChain finishIntake;
    public PathChain scorePreloadPathChainPtTwo;
    public Pose alignGoal2 = new Pose(40, 113, 143);
    public AprilTagDetection lastTagDetected;
    @Override
    public void runOpMode() throws InterruptedException {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        setPathState(-1);
        follower.setStartingPose(startPose);
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
                .addPath(new BezierLine(apTag1, alignGoal2))
                .setLinearHeadingInterpolation(90,143)
                .build();
        this.scorePreloadPathChainPtTwo = follower.pathBuilder()
                .addPath(new BezierLine(alignGoal2, apTag1))
                .setLinearHeadingInterpolation(143,90)
                .build();
        this.detectionPathChain = follower.pathBuilder()
                .addPath(new BezierCurve(Arrays.asList(startPose, new Pose(50,10,robot.hardwareRobot.getHeading()), apTag1)))
                .setConstantHeadingInterpolation(90)
                .build();
        this.pickupPathChain1 = follower.pathBuilder()
                .addPath(new BezierCurve(Arrays.asList(apTag1, new Pose(70,100,90), pickup)))
                .setLinearHeadingInterpolation(robot.hardwareRobot.getHeading(), 180)
                .build();
        this.pickupPathChain2 = follower.pathBuilder()
                .addPath(new BezierLine(apTag1, new Pose(40, 61,0)))
                .setLinearHeadingInterpolation(robot.hardwareRobot.getHeading(), 180)
                .build();
        this.pickupPathChain3 = follower.pathBuilder()
                .addPath(new BezierLine(apTag1, new Pose(40, 35, 0)))
                .setLinearHeadingInterpolation(robot.hardwareRobot.getHeading(), 180)
                .build();
        this.scorePathChain = follower.pathBuilder()
                .addPath(new BezierCurve(Arrays.asList(follower.getPose(), new Pose(70,80, 0), alignGoal)))
                .setLinearHeadingInterpolation(robot.hardwareRobot.getHeading(), 143)
                .build();
        this.returnPathChain = follower.pathBuilder()
                .addPath(new BezierCurve(alignGoal, new Pose(90,20,0), new Pose(10,10,0)))
                .setLinearHeadingInterpolation(143,0)
                .build();
        this.intakePathChain = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), new Pose(follower.getPose().getX() - 6, follower.getPose().getY(), 180)))
                .setConstantHeadingInterpolation(180)
                .build();
        this.finishIntake = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), new Pose(follower.getPose().getX() + 18, follower.getPose().getY(), 180)))
                .setConstantHeadingInterpolation(180)
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
                    if (robot.decode(lastTagDetected).equals("PPG")) follower.followPath(pickupPathChain3);
                    else if (robot.decode(lastTagDetected).equals("PGP")) follower.followPath(pickupPathChain2);
                    else follower.followPath(pickupPathChain1);
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
}

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

@Autonomous (name = "2MagCRight")
public class TwoMagCloseRight extends LinearOpMode {
    public RobotSystem robot = new RobotSystem(hardwareMap, this);
    public PathChain shootPreload;
    public PathChain pickupPathOne;
    public PathChain finishPickupOne;
    public PathChain scorePickupOne;
    public PathChain pickupPathTwo;
    public PathChain finishPickupPathTwo;
    public PathChain scorePickupTwo;
    public PathChain returnPathChain;
    public Follower follower;
    public Timer pathTimer, opmodeTimer;
    public int pathState;
    public final Pose startPose = new Pose(88,6,Math.toRadians(90));
    public final Pose preloadShoot = new Pose(88,6,Math.toRadians(70));
    public Pose pickupOne = new Pose(110, 29, Math.toRadians(0));
    public Pose pickupOneFinish = new Pose(122,29, Math.toRadians(0));
    public Pose alignGoal = new Pose(90, 95, Math.toRadians(36));
    public Pose controlPointOuttakeOne = new Pose(70,50,Math.toRadians(0));
    public final Pose pickupTwo = new Pose(110,55, Math.toRadians(0));
    public final Pose pickupTwoFinish = new Pose(122,55,Math.toRadians(0));
    public final Pose scorePickupTwoControl = new Pose(100,50,Math.toRadians(0));
    public final Pose finish = new Pose(137,5, Math.toRadians(180));
    public final Pose finishControlPoint = new Pose(80,30,Math.toRadians(0));
    public AprilTagDetection lastTagDetected;
    @Override
    public void runOpMode() throws InterruptedException {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        setPathState(0);
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
        this.shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, preloadShoot))
                .setLinearHeadingInterpolation(startPose.getHeading(),preloadShoot.getHeading())
                .build();
        this.pickupPathOne = follower.pathBuilder()
                .addPath(new BezierLine(preloadShoot, pickupOne))
                .setLinearHeadingInterpolation(preloadShoot.getHeading(),pickupOne.getHeading())
                .build();
        this.finishPickupOne = follower.pathBuilder()
                .addPath(new BezierLine(pickupOne, pickupOneFinish))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        this.scorePickupOne = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickupOneFinish,
                        controlPointOuttakeOne,
                        alignGoal
                ))
                .setLinearHeadingInterpolation(pickupOneFinish.getHeading(), alignGoal.getHeading())
                .build();
        this.pickupPathTwo = follower.pathBuilder()
                .addPath(new BezierLine(alignGoal, pickupTwo))
                .setLinearHeadingInterpolation(alignGoal.getHeading(), pickupTwo.getHeading())
                .build();
        this.finishPickupPathTwo = follower.pathBuilder()
                .addPath(new BezierLine(pickupTwo, pickupTwoFinish))
                .setConstantHeadingInterpolation(pickupTwo.getHeading())
                .build();
        this.scorePickupTwo = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickupTwoFinish,
                        scorePickupTwoControl,
                        alignGoal
                ))
                .setLinearHeadingInterpolation(pickupTwoFinish.getHeading(), alignGoal.getHeading())
                .build();
        this.returnPathChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        alignGoal,
                        finishControlPoint,
                        finish
                ))
                .setLinearHeadingInterpolation(alignGoal.getHeading(), finish.getHeading())
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
            }
        } else {
            lastTagDetected = null; // clear old tag when none detected
        }
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) follower.followPath(shootPreload);
                robot.inDep.unloadMag();
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(pickupPathOne);
                    robot.inDep.setIntake(0.6);
                }
                setPathState(2);
                break;
            case 2:
                if (!follower.isBusy()) follower.followPath(finishPickupOne);
                robot.inDep.setIntake(0);
                setPathState(3);
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickupOne);
                }
                robot.inDep.unloadMag();
                setPathState(4);
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(pickupPathTwo);
                }
                robot.inDep.setIntake(0.6);
                setPathState(5);
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(finishPickupPathTwo);
                }
                robot.inDep.setIntake(0);
                setPathState(6);
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickupTwo);
                }
                robot.inDep.unloadMag();
                setPathState(7);
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(returnPathChain);
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
    private Pose getRobotPoseFromCamera(AprilTagDetection tag) {
        return new Pose(tag.robotPose.getPosition().x, tag.robotPose.getPosition().y, follower.getHeading(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}

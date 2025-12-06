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


//TODO: tune drive constants for getting to pos
//TODO: experiment with holdend

@Autonomous (name = "FINALCLAUTO")
public class FRTwoMagCloseLeft extends LinearOpMode {
    public RobotSystem robot;
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
    public final Pose startPose = new Pose(56,8,Math.toRadians(90));
    public final Pose startPosAfter = new Pose(56,28);
    public Pose pickupOne = new Pose(40, 36, Math.toRadians(180));
    public Pose pickupOneFinish = new Pose(23,36, Math.toRadians(180));
    public Pose alignGoal = new Pose(56, 11, Math.toRadians(117));
    public final Pose pickupTwo = new Pose(40,60, Math.toRadians(180));
    public final Pose pickupTwoFinish = new Pose(25,60,Math.toRadians(180));
    public final Pose finish = new Pose(12,10, Math.toRadians(0));
    public AprilTagDetection lastTagDetected;
    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        setPathState(0);
        follower.setStartingPose(startPose);
        buildPaths();
        robot.inDep.initControllers();
        waitForStart();
        while (opModeIsActive()) {
            detectTags();
            follower.update();
            autonomousPathUpdate();
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
    public boolean xInchRadius(int radius, AprilTagDetection target) {
        return target.ftcPose.range <= radius;
    }
    public void buildPaths() {
        this.shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, startPosAfter))
                .addPath(new BezierLine(startPosAfter, alignGoal))
                .setLinearHeadingInterpolation(startPose.getHeading(),alignGoal.getHeading() - Math.toRadians(4.5))
                .build();
        this.pickupPathOne = follower.pathBuilder()
                .addPath(new BezierLine(alignGoal, pickupOne))
                .setLinearHeadingInterpolation(alignGoal.getHeading(),pickupOne.getHeading())
                .build();
        this.finishPickupOne = follower.pathBuilder()
                .addPath(new BezierLine(pickupOne, pickupOneFinish))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        this.scorePickupOne = follower.pathBuilder()
                .addPath(new BezierLine(pickupOneFinish, alignGoal))
//                .addPath(new BezierLine(alignGoal, startPosAfter))
//                .addPath(new BezierLine(startPosAfter, alignGoal))
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
                .addPath(new BezierLine(pickupTwoFinish, alignGoal))
                .setLinearHeadingInterpolation(pickupTwoFinish.getHeading(), alignGoal.getHeading())
                .build();
        this.returnPathChain = follower.pathBuilder()
                .addPath(new BezierLine(alignGoal, finish))
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
    boolean pathF1 = true;
    boolean pathF2 = true;
    boolean pathF3 = true;
    boolean pathF4 = true;
    boolean pathF5 = true;
    boolean pathF6 = true;
    boolean pathF7 = true;
    boolean pathF8 = true;
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (pathF1) {
                    follower.setMaxPower(0.45);
                    follower.followPath(shootPreload, true);
                    robot.inDep.toggleControlServo(0,0.31);
                    pathF1 = false;
                }
                if (follower.atPose(alignGoal,3,3,Math.toRadians(1)) || pathTimer.getElapsedTimeSeconds() > 0.85) {
                    setPathState(-2);
                }
                break;
            case -2:
                if (!follower.isBusy()) {
                    if (follower.atPose(alignGoal,3,3,Math.toRadians(3)) || pathTimer.getElapsedTimeSeconds() > 0.85) {
                        robot.inDep.unloadMag(opmodeTimer);
                        robot.inDep.setShooterPower(0);
                        robot.inDep.resetUnloadMacro();
                        setPathState(1);
                    }
                }
                break;
            case 1:
                if (pathF2){
                    follower.setMaxPower(0.55);
                    follower.followPath(pickupPathOne, true);
                    pathF2 = false;
                }
                if(follower.atPose(pickupOne,3,3,Math.toRadians(3))) {
                    robot.inDep.setIntake(0.6);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()){
                    if (pathF3){
                        follower.followPath(finishPickupOne);
                        pathF3 = false;
                    }
                    if (follower.atPose(pickupOneFinish, 3,3, Math.toRadians(3)) || pathTimer.getElapsedTimeSeconds() > 0.85) {
                        if(pathTimer.getElapsedTimeSeconds() > 1.8) {
                            robot.inDep.setIntake(0);
                            setPathState(3);
                        }
                    }
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    if (pathF4){
                        follower.setMaxPower(0.65);
                        follower.followPath(scorePickupOne, true);
                        pathF4 = false;
                    }
                    if(follower.atPose(alignGoal, 3,3, Math.toRadians(3))) {
                        robot.inDep.unloadMag(opmodeTimer);
                        robot.inDep.setShooterPower(0);
                        robot.inDep.resetUnloadMacro();
                        robot.inDep.setIntake(0);
                        setPathState(7);
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    if (pathF5){
                        follower.followPath(pickupPathTwo);
                        pathF5 = false;
                    }
                    if (follower.atPose(pickupTwo, 3,3, Math.toRadians(3))) {
                        robot.inDep.setIntake(0.6);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if (!follower.isBusy()){
                    if (pathF6){
                        follower.followPath(finishPickupPathTwo);
                        pathF6 = false;
                    }
                    if (follower.atPose(pickupTwoFinish, 3,3, Math.toRadians(3))) {
                        robot.inDep.setIntake(0);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if (!follower.isBusy()){
                    if (pathF7){
                        follower.followPath(scorePickupTwo, true);
                        pathF7 = false;
                    }
                    if (follower.atPose(alignGoal, 3,3,Math.toRadians(3))) {
                        robot.inDep.unloadMag(opmodeTimer);
                        robot.inDep.setShooterPower(0);
                        robot.inDep.resetUnloadMacro();
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    if (pathF8){
                        follower.followPath(returnPathChain);
                        pathF8 = false;
                    }
                    if (follower.atPose(finish, 3,3,Math.toRadians(1))) {
                        setPathState(8);
                    }
                }
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
    public boolean atPose(Pose pos, double xTol, double yTol, double angleTol) {
        boolean pose = follower.atPose(pos, xTol, yTol);
        double angle = follower.getHeadingError();
        boolean ang = (angle == 0);
        return pose && ang;
    }
}

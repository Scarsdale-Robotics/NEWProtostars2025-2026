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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.ArrayList;


//add pathtimer.getelapsedtimeseconds > x for failsafes incase atpose doesnt work

@Autonomous (name = "FINALCLAUTO")
public class PeekskillCL extends LinearOpMode {
    public RobotSystem robot;
    public PathChain shootPreload;
    public PathChain pickupPathOne;
    public PathChain finishPickupOne;
    public PathChain scorePickupOne;
    public PathChain pickupPathTwo;
    public PathChain finishPickupPathTwo;
    public PathChain scorePickupTwo;
    public PathChain pickupPathThree;
    public PathChain finishPickupPathThree;
    public PathChain scorePickupThree;
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
    public final Pose pickupTwoFinish = new Pose(23,60,Math.toRadians(180));
    public final Pose pickupThree = new Pose(40,84, Math.toRadians(180));
    public final Pose pickupThreeFinish = new Pose(23,84, Math.toRadians(180));
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
        waitForStart();
        while (opModeIsActive()) {
            //detectTags();
            follower.update();
            autonomousPathUpdate();
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
    public void buildPaths() {
        this.shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, startPosAfter))
                .setConstantHeadingInterpolation(startPose.getHeading())
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
        this.pickupPathThree = follower.pathBuilder()
                .addPath(new BezierLine(alignGoal, pickupThree))
                .setLinearHeadingInterpolation(alignGoal.getHeading(), pickupThree.getHeading())
                .build();
        this.finishPickupPathThree = follower.pathBuilder()
                .addPath(new BezierLine(pickupThree, pickupThreeFinish))
                .setConstantHeadingInterpolation(pickupThree.getHeading())
                .build();
        this.scorePickupThree = follower.pathBuilder()
                .addPath(new BezierLine(pickupThreeFinish, alignGoal))
                .setLinearHeadingInterpolation(pickupThreeFinish.getHeading(), alignGoal.getHeading())
                .build();
        this.returnPathChain = follower.pathBuilder()
                .addPath(new BezierLine(alignGoal, finish))
                .setLinearHeadingInterpolation(alignGoal.getHeading(), finish.getHeading())
                .build();
    }
    boolean pathF1 = true;
    boolean pathF2 = true;
    boolean pathF3 = true;
    boolean pathF4 = true;
    boolean pathF5 = true;
    boolean pathF6 = true;
    boolean pathF7 = true;
    boolean pathF8 = true;
    boolean pathF9 = true;
    boolean pathF10 = true;
    boolean pathF11 = true;
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (pathF1) {
                    follower.followPath(shootPreload, true);
                    pathF1 = false;
                }
                if (follower.atPose(alignGoal,3,3,Math.toRadians(1))) {
                    setPathState(-2);
                }
                break;
            case -2:
                if (!follower.isBusy()) {
                    if (follower.atPose(alignGoal,3,3,Math.toRadians(3))) {
                        robot.inDep.unloadMag(opmodeTimer);
                        robot.inDep.setShooterPower(0);
                        robot.inDep.resetUnloadMacro();
                        setPathState(1);
                    }
                }
                break;
            case 1:
                if (pathF2){
                    follower.followPath(pickupPathOne, true);
                    pathF2 = false;
                }
                if(follower.atPose(pickupOne,3,3,Math.toRadians(3))) {
                    robot.inDep.setTransfer(0.6);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()){
                    if (pathF3){
                        follower.followPath(finishPickupOne);
                        pathF3 = false;
                    }
                    if (follower.atPose(pickupOneFinish, 3,3, Math.toRadians(3))) {
                        robot.inDep.setTransfer(0);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    if (pathF4){
                        follower.followPath(scorePickupOne, true);
                        pathF4 = false;
                    }
                    if(follower.atPose(alignGoal, 3,3, Math.toRadians(3))) {
                        robot.inDep.unloadMag(opmodeTimer);
                        robot.inDep.setShooterPower(0);
                        robot.inDep.resetUnloadMacro();
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    if (pathF5){
                        follower.followPath(pickupPathTwo, true);
                        pathF5 = false;
                    }
                    if (follower.atPose(pickupTwo, 3,3, Math.toRadians(3))) {
                        robot.inDep.setTransfer(0.6);
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
                        robot.inDep.setTransfer(0);
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
                if (!follower.isBusy()) {
                    if (pathF8) {
                        follower.followPath(pickupPathThree, true);
                        pathF8 = false;
                    }
                    if (follower.atPose(pickupThree, 3,3,Math.toRadians(3))) {
                        robot.inDep.setTransfer(0.6);
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    if (pathF9) {
                        follower.followPath(finishPickupPathThree);
                        pathF9 = false;
                    }
                    if (follower.atPose(pickupThreeFinish,3,3,Math.toRadians(3))) {
                        robot.inDep.setTransfer(0);
                        setPathState(9);
                    }
                }
                break;
            case 9:
                if (!follower.isBusy()){
                    if (pathF10){
                        follower.followPath(scorePickupThree, true);
                        pathF10 = false;
                    }
                    if (follower.atPose(alignGoal, 3,3,Math.toRadians(3))) {
                        robot.inDep.unloadMag(opmodeTimer);
                        robot.inDep.setShooterPower(0);
                        robot.inDep.resetUnloadMacro();
                        setPathState(10);
                    }
                }
                break;
            case 10:
                if (!follower.isBusy()){
                    if (pathF11){
                        follower.followPath(returnPathChain);
                        pathF11 = false;
                    }
                    if (follower.atPose(finish, 3,3,Math.toRadians(1))) {
                        setPathState(11);
                    }
                }
                break;
            case 11:
                if (!follower.isBusy()) setPathState(-1);
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}

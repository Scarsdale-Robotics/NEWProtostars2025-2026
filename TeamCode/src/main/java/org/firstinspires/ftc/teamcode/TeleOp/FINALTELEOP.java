package org.firstinspires.ftc.teamcode.TeleOp;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
@TeleOp (name = "TeleOp")
//TODO: add toggle logic for control servo, make shooting macro
//TODO: add shooting controls, intake controls.

public class FINALTELEOP extends LinearOpMode {
    public PathChain path;
    public RobotSystem robot;
    public Follower follower;
    public Timer pathTimer, opModeTimer;
    public AprilTagDetection lastTagDetected;
    public double speed;
    public Pose startPose;
    public boolean lastSqPressed = false;
    public Pose alignGoal;
    @Override
    public void runOpMode() throws InterruptedException {
        this.startPose = new Pose(0,0,Math.toRadians(0));
        this.alignGoal = new Pose(0,0,Math.toRadians(0));
        //for the above, make it so square = closeLeft, circle = closeRight, triangle = farLeft, cross = farRight
        //same for alignGoal. except square = blue goal, circle = red goal
        this.pathTimer = new Timer();
        this.opModeTimer = new Timer();
        pathTimer.resetTimer();
        robot.hardwareRobot.initOdom();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        opModeTimer.resetTimer();
        this.robot = new RobotSystem(hardwareMap, this);
        this.speed = 0.5;
        while (opModeIsActive()) {
            detectTags();
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            robot.drive.driveRobotCentricPowers(strafe, forward, turn);
            boolean square = gamepad1.square;
            if (!lastSqPressed && square) {
               bezierLine(follower.getPose(), alignGoal);
            }
            lastSqPressed = square;
        }
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
                telemetry.addData("Range", tag.ftcPose.range);
                lastTagDetected = tag;
                break;
            }
        } else {
            lastTagDetected = null; // clear old tag when none detected
        }
    }
    public boolean xInchRadius(AprilTagDetection target, int radius) {
        return target.ftcPose.range <= radius;
    }
    public void bezierLine(Pose current, Pose target) {
        this.path = follower.pathBuilder()
                .addPath(new BezierLine(current, target))
                .setLinearHeadingInterpolation(current.getHeading(), target.getHeading())
                .build();
        follower.followPath(path);
        robot.inDep.unloadMag();
    }
}

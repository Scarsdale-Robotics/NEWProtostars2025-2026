package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class PedroPathingTeleop extends LinearOpMode {
    public RobotSystem robot;
    public AprilTagDetection lastTagDetected;
    public Timer pathTimer, opModeTimer;
    public double speed;
    public PathChain score;
    public Follower follower;
    public boolean sqLastPressed = false;
    public int pathState = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        this.pathTimer = new Timer();
        this.opModeTimer = new Timer();
        pathTimer.resetTimer();
        this.follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        opModeTimer.resetTimer();
        this.robot = new RobotSystem(hardwareMap, this);
        this.speed = 0.5;
        while (opModeIsActive()) {
            detectTags();
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            if (!follower.isBusy()) robot.drive.driveRobotCentricPowers(strafe,forward,turn);
            boolean square = gamepad1.square;
            if (square && !sqLastPressed) {
                setPathState(1);
                follower.followPath(score);
                setPathState(0);
            }
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
                telemetry.addData("Range: ", tag.ftcPose.range);
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
    public void buildPaths() {
        this.score = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(lastTagDetected.robotPose.getPosition().x, lastTagDetected.robotPose.getPosition().y, robot.hardwareRobot.getHeading(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE),
                        new Pose(120,130,0),
                        new Pose(27,130)
                ))
                .setLinearHeadingInterpolation(robot.hardwareRobot.getHeading(), 143)
                .build();
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}

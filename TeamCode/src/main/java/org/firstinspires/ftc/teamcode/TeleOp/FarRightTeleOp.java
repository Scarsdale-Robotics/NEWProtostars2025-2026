package org.firstinspires.ftc.teamcode.TeleOp;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.telemetry.PanelsTelemetry;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.Subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp(name = "FRTele")

public class FarRightTeleOp extends LinearOpMode {
    public RobotSystem robot;
    public AprilTagDetection lastTagDetected;
    public double speed;
    public Timer opModeTimer;
    public boolean lastToggleShootMacro = false;
    public boolean lastShooter = false;
    public Timer pathTimer;
    public static int pathState;
    public Follower follower;
    public Pose startPose;
    public Pose target;
    public PathChain path;
    public Pose alignGoal;

    @Override
    public void runOpMode() throws InterruptedException {
        this.follower = Constants.createFollower(hardwareMap);
        this.opModeTimer = new Timer();
        this.robot = new RobotSystem(hardwareMap, this);
        this.pathTimer = new Timer();
        robot.inDep.setShooterPower(0);
        robot.hardwareRobot.initOdom();
        robot.inDep.initAutoAim(false,true);
        robot.inDep.initControllers();
        this.speed = 0.6;
        this.startPose = new Pose(115,134, Math.toRadians(270));
        this.alignGoal = new Pose(85,90, Math.toRadians(43));
        follower.setStartingPose(startPose);
        opModeTimer.resetTimer();
        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            detectTags();
            double strafe = gamepad1.left_stick_x;
            double forward = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            telemetry.addData("heading", robot.hardwareRobot.pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("X", robot.hardwareRobot.pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y", robot.hardwareRobot.pinpoint.getPosY(DistanceUnit.INCH));
            robot.drive.driveRobotCentricPowers(strafe * speed, forward * speed, turn * speed);
            boolean intakePressed = gamepad1.left_bumper;
            if (intakePressed) robot.inDep.setIntake(0.75);
            else if (gamepad1.right_bumper) robot.inDep.setIntake(-0.75);
            else robot.inDep.setIntake(0);
            boolean transferPressed = gamepad1.triangle;
            if (transferPressed) robot.inDep.setTransfer(0.65);
            else if (gamepad1.circle) robot.inDep.setTransfer(-0.65);
            else robot.inDep.setTransfer(0);
            boolean shooter = gamepad1.dpad_up;
            if (shooter && !lastShooter) robot.inDep.time = null;
            if (shooter) robot.inDep.setShooterVelocity(1860);
            else {
                robot.inDep.setShooterPower(0);
            }
            robot.inDep.autoAim(follower);
            telemetry.addData("Corrected Heading", normalizeAngle(robot.hardwareRobot.pinpoint.getHeading(AngleUnit.DEGREES)));
            boolean toggleShooter = gamepad1.options;
            if (!lastToggleShootMacro && toggleShooter) robot.inDep.unloadMag(opModeTimer);
            lastToggleShootMacro = toggleShooter;
            lastShooter = shooter;
            telemetry.update();
            PanelsTelemetry.INSTANCE.getTelemetry().update();
        }
    }

    public void detectTags() {
        ArrayList<AprilTagDetection> detections = robot.cv.aprilTagProcessor.getDetections();

        if (detections != null) {
            for (AprilTagDetection tag : detections) {
                if (tag.rawPose != null) {
                    telemetry.addLine("AprilTag Detected.");
                    telemetry.addData("ID", tag.id);
                    telemetry.addData("X (Sideways offset)", tag.ftcPose.x);
                    telemetry.addData("Y (Forward/Back Offset)", tag.ftcPose.y);
                    telemetry.addData("Z", tag.ftcPose.z);
                    telemetry.addData("Bearing", tag.ftcPose.bearing);
                    telemetry.addData("Yaw", tag.ftcPose.yaw);
                    telemetry.addData("Range", tag.ftcPose.range);
                }
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

    public AprilTagDetection getTargetTag(int id1, int id2) {
        ArrayList<AprilTagDetection> detections = robot.cv.aprilTagProcessor.getDetections();
        for (AprilTagDetection tag : detections) {
            if (tag.id == id1 || tag.id == id2) {
                return tag;
            }
        }
        return null;
    }
    public double normalizeAngle(double angle) {
        if (angle <= 0) return Math.abs(angle);
        else if (angle > 0) return (180 - angle) + 180;
        return 0;
    }
    public void setPathState (int pstate) {
        pathTimer.resetTimer();
        pathState = pstate;
    }
    public void DTP(Pose pose) {
        Pose current = follower.getPose();
        this.path = follower.pathBuilder()
                .addPath(new BezierLine(current, pose))
                .setLinearHeadingInterpolation(current.getHeading(), pose.getHeading())
                .build();
        follower.followPath(path);
        boolean p1 = true;
        while (opModeIsActive()) {
            if (p1) {
                follower.followPath(path);
                p1 = false;
            }
            if (follower.atPose(pose, 2,2,Math.toRadians(2))) break;
        }
    }
}

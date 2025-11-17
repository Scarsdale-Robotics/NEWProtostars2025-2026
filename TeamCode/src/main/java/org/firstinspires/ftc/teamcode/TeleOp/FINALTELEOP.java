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

@TeleOp(name = "TeleOp")

public class FINALTELEOP extends LinearOpMode {
    public RobotSystem robot;
    public AprilTagDetection lastTagDetected;
    public double speed;
    public Pose startPose;
    public boolean intakePressed = false;
    public boolean lastToggleServoPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.hardwareRobot.initOdom();
        robot.inDep.setShooterPower(1);
        this.robot = new RobotSystem(hardwareMap, this);
        this.speed = 0.7;
        waitForStart();
        while (opModeIsActive()) {
            detectTags();

            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            robot.drive.driveRobotCentricPowers(strafe, forward, turn);
            if (intakePressed) robot.inDep.setIntake(0.7);
            else robot.inDep.setIntake(0);
            boolean toggleServo = gamepad1.triangle;
            if (!lastToggleServoPressed && toggleServo) {
                robot.inDep.toggleControlServo(0,1);
            }
            lastToggleServoPressed = toggleServo;
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
}

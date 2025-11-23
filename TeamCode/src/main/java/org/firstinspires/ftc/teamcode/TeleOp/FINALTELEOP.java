package org.firstinspires.ftc.teamcode.TeleOp;
import com.acmerobotics.roadrunner.util.Angle;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.Subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp(name = "TeleOp - JJ Scrim")

public class FINALTELEOP extends LinearOpMode {
    public RobotSystem robot;
    public AprilTagDetection lastTagDetected;
    public double speed;
    public boolean intakeOnePressed = false;
    public boolean intakeTwoPressed = false;

    public boolean lastToggleServoPressed = false;
    public boolean shooterPressed = false;
    public Timer opModeTimer;
    public boolean lastToggleShootMacro = false;
    public boolean lastToggleAlignMacro = false;

    @Override
    public void runOpMode() throws InterruptedException {
        this.opModeTimer = new Timer();
        this.robot = new RobotSystem(hardwareMap, this);
        robot.inDep.setShooterPower(0);
        robot.hardwareRobot.initOdom();
        this.speed = 0.6;
        opModeTimer.resetTimer();
        waitForStart();
        while (opModeIsActive()) {
            robot.hardwareRobot.pinpoint.update();
            detectTags();
            double strafe = gamepad1.left_stick_x;
            double forward = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            telemetry.addData("heading", robot.hardwareRobot.pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("X", robot.hardwareRobot.pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y", robot.hardwareRobot.pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.update();
            robot.drive.driveRobotCentricPowers(strafe * speed, forward * speed, turn * speed);
            intakeOnePressed = gamepad1.left_bumper;
            if (intakeOnePressed) robot.inDep.setFrontIn(0.75);
            else if (gamepad1.right_bumper) robot.inDep.setFrontIn(-0.75);
            else robot.inDep.setFrontIn(0);
            intakeTwoPressed = gamepad1.triangle;
            if (intakeTwoPressed) robot.inDep.setSecondIn(0.75);
            else if (gamepad1.cross) robot.inDep.setSecondIn(-0.75);
            else robot.inDep.setSecondIn(0);
            //boolean toggleAlign = gamepad2.triangle;
            //if (toggleAlign && !lastToggleAlignMacro) {
              //  alignToTag(getTargetTag(20, 24));
            //}
            boolean toggleServo = gamepad1.square;
            if (!lastToggleServoPressed && toggleServo) {
                robot.inDep.toggleControlServo(0,0.31);
                if (robot.hardwareRobot.intakeControl.getPosition() == 0) gamepad1.setLedColor(0,255,0,-1);
                else gamepad1.setLedColor(255,0,0,-1);
            }
            else if (gamepad1.dpad_up) robot.inDep.setShooterPower(0.8);
            else if (gamepad1.dpad_right) robot.inDep.setShooterPower(0.75);
            else if (gamepad1.dpad_down) robot.inDep.setShooterPower(0.7);
            else if (gamepad1.dpad_left) robot.inDep.setShooterPower(0.87);
            else {
                if (gamepad2.cross) robot.inDep.setShooterPower(0);
                robot.inDep.setShooterPower(0.3);
            }
            //boolean toggleShooter = gamepad1.options;
            //if (!lastToggleShootMacro && toggleShooter) robot.inDep.unloadMag(opModeTimer);
            lastToggleServoPressed = toggleServo;
            //lastToggleShootMacro = toggleShooter;
            //lastToggleAlignMacro = toggleAlign;
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
    public void alignToTag(AprilTagDetection tag) {
        PIDController pid = new PIDController(0.4,0.01,0.005);
        pid.setTolerance(0.8);
        pid.setSetPoint(0);
        while (opModeIsActive() && !pid.atSetPoint()) {
            double error = 0 - tag.ftcPose.bearing;
            if (Math.abs(error) <= 0.8) break;
            double power = pid.calculate(error, 0);
            robot.inDep.clamp(power);
            robot.drive.driveRobotCentricPowers(0,0,power);
        }
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
}

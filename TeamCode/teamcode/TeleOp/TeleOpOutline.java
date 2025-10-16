package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.Auto.SimpleAuto;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
//TODO: implement to intake next system
public class TeleOpOutline extends LinearOpMode {
    public double speed;
    public RobotSystem robot;
    public AprilTagDetection lastTagDetected;
    public char first;
    public char second;
    public char third;
    public boolean fr;
    public boolean sc;
    public boolean lastSquarePressed;
    public Pose2d currentPose;
    public boolean tr;
    public String motif;
    @Override
    public void runOpMode() throws InterruptedException {
        this.currentPose = new Pose2d(0,0,0);
        this.robot = new RobotSystem(hardwareMap, this);
        robot.hardwareRobot.setImu();
        motif = SimpleAuto.motif;
        this.speed = 0.5;
        first = motif.charAt(0);
        second = motif.charAt(1);
        third = motif.charAt(2);
        waitForStart();
        while (opModeIsActive()) {
            robot.hardwareRobot.pinpoint.update();
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            if (xInchRadius(lastTagDetected, 10)) {
                speed = 0.3;
            } else {
                speed = 0.5;
            }
            robot.drive.driveFieldCentricPowers(strafe * speed, forward * speed, turn * speed, robot.hardwareRobot.getHeading());
            detectTags();
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Forward", forward);
            telemetry.addData("Turn", turn);
            if (!fr) {
                telemetry.addLine("Next" + first);
            } else {
                if (!sc) {
                    telemetry.addLine("Next:" + second);
                } else {
                    telemetry.addLine("Next:" + third);
                }
            }
            telemetry.update();
            if (gamepad1.square && !lastSquarePressed) {

            }
            lastSquarePressed = gamepad1.square;
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
}

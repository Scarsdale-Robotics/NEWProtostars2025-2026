package org.firstinspires.ftc.teamcode.TeleOp;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierCurve;
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
/* Using the fieldDrive method, you can follow a bezier line between your position and other points on the field.
 * This is primarily for macros and easier scoring.
 */
public class FINALTELEOP extends LinearOpMode {
    public RobotSystem robot;
    public AprilTagDetection lastTagDetected;
    public Timer pathTimer, opModeTimer;
    public double speed;
    public boolean lastSqPressed = false;
    @Override
    public void runOpMode() throws InterruptedException {
        this.pathTimer = new Timer();
        this.opModeTimer = new Timer();
        pathTimer.resetTimer();
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
                fieldDrive(lastTagDetected, 100,100,30);
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
    public void fieldDrive(AprilTagDetection target, int xCoordinate, int yCoordinate, double targetHeading) {
        PIDController tagController = new PIDController(0.02,0,0.001);
        tagController.setTolerance(0.1);
        while (opModeIsActive() && !tagController.atSetPoint()) {
            double powerTurn = tagController.calculate(robot.hardwareRobot.pinpoint.getHeading(AngleUnit.DEGREES), targetHeading);
            double powerX = tagController.calculate(target.robotPose.getPosition().x, xCoordinate);
            double powerY = tagController.calculate(target.robotPose.getPosition().y, yCoordinate);
            robot.drive.driveRobotCentricPowers(powerX, powerY, powerTurn);
        }
    }
}

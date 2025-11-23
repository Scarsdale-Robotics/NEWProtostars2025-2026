package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
@TeleOp (name = "PIDTEST")
public class AlignTTTest extends LinearOpMode {
    public AprilTagDetection lastTagDetected;
    public RobotSystem robot;
    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        while (opModeIsActive()) {
            detectTags();
            alignToTag(getTargetTag(20,24));
        }
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
}

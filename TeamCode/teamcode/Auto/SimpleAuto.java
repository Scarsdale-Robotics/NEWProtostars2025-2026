package org.firstinspires.ftc.teamcode.Auto;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotserver.internal.webserver.RobotControllerWebHandlers;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class SimpleAuto extends LinearOpMode {
    public static String motif;
    public RobotSystem robot;
    public AprilTagDetection lastTagDetected;
    public int iterations = 0;
    public String sequence;
    //TODO: implement 2 wheel odometry + imu and pose2d + roadrunner. Also, create hybrid system w apriltags as well
    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        robot.hardwareRobot.setImu();
        waitForStart();
        while (opModeIsActive() && iterations < 2) {
            driveToTag(lastTagDetected, 100, 100);
            //rely on odometry here - three sequences based off of motif
            sequence = robot.decode(lastTagDetected);
            turn(30);
            //outtake
            turn(60);
            odomDrive(1000);
            turn(90);
            sequence(sequence);
            driveToTag(lastTagDetected, 300, 100);
            //outtake
            motif = sequence;
            iterations++;
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
    //TODO: tweak coordinates for actual placement in front of apriltag
    public void driveToTag(AprilTagDetection target, int xCoordinate, int yCoordinate) {
        detectTags();
        PIDController tagController = new PIDController(0.02,0,0.001);
        tagController.setTolerance(1);
        while (opModeIsActive() && !tagController.atSetPoint()) {
            double powerX = tagController.calculate(target.robotPose.getPosition().x, xCoordinate);
            double powerY = tagController.calculate(target.robotPose.getPosition().y, yCoordinate);
            robot.drive.driveRobotCentricPowers(powerX, powerY, 0);
        }
    }
    //ticks
    public void odomDrive(double inches) {
        detectTags();
        PIDController controllerOdom = new PIDController(0.02,0,0.001);
        controllerOdom.setTolerance(1);
        int targetTicks = (int)(inches * new RobotConstants().TICKS_PER_INCH);
        while (opModeIsActive() && !controllerOdom.atSetPoint()) {
            double power = controllerOdom.calculate(robot.hardwareRobot.pinpoint.getEncoderY(), targetTicks);
            robot.drive.driveRobotCentricPowers(0, power, 0);
        }
    }
    public void turn(int degrees) {
        detectTags();
        PIDController controller = new PIDController(0.02, 0, 0.001);  // tune these
        controller.setTolerance(1);

        double targetAngle = robot.hardwareRobot.getHeading() + degrees;

        while (opModeIsActive() && !controller.atSetPoint()) {
            double power = controller.calculate(robot.hardwareRobot.getHeading(), targetAngle);
            robot.drive.driveRobotCentricPowers(0, 0, power);
            telemetry.addData("Heading", robot.hardwareRobot.getHeading());
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }


    public boolean xInchRadius(int radius, AprilTagDetection target) {
        return target != null && target.ftcPose.range <= radius;
    }
    public void sequence(String motif) {
        if (motif.equals("GPP")) {
            odomDrive(1000);
        }
        else if (motif.equals("PGP")) {
            odomDrive(2000);
        } else {
            odomDrive(3000);
        }
        for (int i = 0; i < 3; i++) {
            odomDrive(100);
            //intake
        }
        turn(-90);
    }
}
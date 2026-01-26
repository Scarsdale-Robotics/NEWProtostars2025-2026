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
@TeleOp(name = "CRTele")

public class CloseRightTeleOp extends LinearOpMode {
    public RobotSystem robot;
    public double speed;
    public Timer opModeTimer;
    public boolean lastToggleShootMacro = false;
    public boolean lastShooter = false;
    public Follower follower;
    public Pose startPose;
    public boolean lastServo = false;

    @Override
    public void runOpMode() throws InterruptedException {
        this.follower = Constants.createFollower(hardwareMap);
        this.opModeTimer = new Timer();
        this.robot = new RobotSystem(hardwareMap, this);
        robot.inDep.initAutoAim(false,false);
        this.speed = 0.8;
        this.startPose = new Pose(88,8.5,Math.toRadians(90));
        follower.setStartingPose(startPose);
        opModeTimer.resetTimer();
        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            telemetry.addData("heading", Math.toRadians(follower.getHeading()));
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            robot.drive.driveRobotCentricPowers(strafe * speed, forward * speed, turn * speed);
            boolean transferPressed = gamepad1.triangle;
            if (transferPressed) robot.inDep.setTransfer(0.65);
            else if (gamepad1.circle) robot.inDep.setTransfer(-0.65);
            else robot.inDep.setTransfer(0);
            robot.inDep.autoAim(follower);
            boolean transferServo = gamepad1.square;
            if (transferServo && !lastServo) robot.inDep.toggleServo();
            //boolean toggleShooter = gamepad1.options;
            //if (!lastToggleShootMacro && toggleShooter) robot.inDep.unloadMag(opModeTimer);
            //lastToggleShootMacro = toggleShooter;
            telemetry.update();
            lastServo = transferServo;
        }
    }
}

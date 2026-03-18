package org.firstinspires.ftc.teamcode.CV;
import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.KalmanFilter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Configurable
@TeleOp(name = "CLTELE 2/28")
public class ExampleKalmanFilter extends LinearOpMode {
    public CameraName cam;
    public VisionPortal vp;
    public AprilTagProcessor ap;
    private final Size CAMERA_RESOLUTION = new Size(640, 480);
    public AprilTagDetection lastTagDetected;
    public Pose kalmanpose = new Pose(0,0,0);
    public static double lastX = 0;
    public static double lastY = 0;
    public static double lastH = 0;
    public KalmanFilter kalmanX;
    public KalmanFilter kalmanY;
    public KalmanFilter kalmanH;
    public Motor rightFront;
    public Motor rightBack;
    public Motor leftFront;
    public Motor leftBack;
    public Motor shooter;
    public Motor shooter2;
    public Servo hoodServo;
    public DriveSubsystem drive;
    public PIDFController pid;
    public PIDController pidTur;
    public Motor turret;
    public Follower follower;
    public FollowerConstants constants;
    public Motor transfer;
    public boolean lastServo = false;
    public Servo servo;
    public static int x = 5;
    public static int y = 141;
    public Pose startPose = new Pose(56,8,Math.toRadians(90));
    public static double apX = 0;
    public static double apY = 0;
    public static double apH = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        this.cam = hardwareMap.get(CameraName.class, "Webcam 1");
        this.kalmanX = new KalmanFilter(0.4, 0.1);
        this.kalmanY = new KalmanFilter(0.4, 0.5);
        this.kalmanH = new KalmanFilter(0.4, 0.4);
        leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        leftBack = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_435);
        rightBack = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_435);
        turret = new Motor(hardwareMap, "turret", Motor.GoBILDA.RPM_312);

        leftFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftFront.setRunMode(Motor.RunMode.RawPower);
        rightFront.setRunMode(Motor.RunMode.RawPower);
        leftBack.setRunMode(Motor.RunMode.RawPower);
        rightBack.setRunMode(Motor.RunMode.RawPower);
        turret.setRunMode(Motor.RunMode.RawPower);

        leftFront.setInverted(true);
        rightFront.setInverted(true);
        leftBack.setInverted(true);
        rightBack.setInverted(true);


        leftFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        this.drive = new DriveSubsystem(
                leftFront,
                rightFront,
                leftBack,
                rightBack
        );
        //TODO: change
        Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
        this.ap = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .build();
        this.vp = new VisionPortal.Builder()
                .setCamera(cam)
                .enableLiveView(true)
                .setCameraResolution(CAMERA_RESOLUTION)
                .addProcessor(ap)
                .setAutoStartStreamOnBuild(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setLiveViewContainerId(hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()))
                .build();
        vp.setProcessorEnabled(ap, true);
        transfer = new Motor(hardwareMap, "transfer", Motor.GoBILDA.RPM_435);
        transfer.setRunMode(Motor.RunMode.RawPower);
        this.servo = hardwareMap.get(Servo.class, "tsservo");
        this.hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        this.pid = new PIDFController(0.0027,0,0,0);
        this.shooter = new Motor(hardwareMap, "shooter", Motor.GoBILDA.RPM_1620);
        this.shooter2 = new Motor(hardwareMap, "shooter2", Motor.GoBILDA.RPM_1620);
        this.pidTur = new PIDController(0.008,0,0);
        turret = new Motor(hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
        //transfer = new Motor(hardwareMap, "transfer", Motor.GoBILDA.RPM_1620);
        turret.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //transfer.motor.setMode(DcMoto
        turret.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //transfer.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setRunMode(Motor.RunMode.RawPower);
        //transfer.setRunMode(Motor.RunMode.RawPower);
        turret.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //transfer.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAK
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooter.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //transfer.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //transfer.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setInverted(true);
        shooter2.setInverted(true);

        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter2.setRunMode(Motor.RunMode.RawPower);
        //transfer.setRunMode(Motor.RunMode.RawPower);

        shooter.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //transfer.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            kalmanX.update(follower.getPose().getX() - lastX, apX);
            kalmanH.update(follower.getPose().getX() - lastH, apH);
            kalmanY.update(follower.getPose().getX() - lastY, apY);
            double strafee = gamepad1.left_stick_x;
            double forwardd = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double speed = 0.5;
            if (gamepad1.triangle) transfer.motor.setPower(-1);
            else if (gamepad1.circle) transfer.motor.setPower(1);
            else transfer.motor.setPower(0);
            if (gamepad1.dpad_up && !lastServo) toggleServo();
            telemetry.addData("Follower X", follower.getPose().getX());
            telemetry.addData("Follower Y", follower.getPose().getY());
            telemetry.addData("Follower H", Math.toDegrees(follower.getHeading()));
            telemetry.addData("distance", Math.sqrt(Math.pow(x - follower.getPose().getX(),2) + Math.pow(y - follower.getPose().getY(),2)));
            drive.controller.driveRobotCentric(strafee * speed, forwardd * speed, turn * speed);
            autoAim(follower);
            shooterVelocityTwo(shooterVelocity(x,y,follower));
            hoodServo.setPosition(hoodAngle(x,y,follower));
            telemetry.update();
            lastServo = gamepad1.dpad_up;
            lastH = follower.getHeading();
            lastX = follower.getPose().getX();
            lastY = follower.getPose().getY();
            kalmanUpdate();
            tagUpdate();
        }
    }
    public double hoodAngle(double x, double y, Follower follower) {
        double dist = Math.sqrt(Math.pow(x - follower.getPose().getX(),2) + Math.pow(y - follower.getPose().getY(), 2));
        return (-0.000687 * dist) + 0.207;
    }
    public double shooterVelocity(double x, double y, Follower follower) {
        double dist = Math.sqrt(Math.pow(x - follower.getPose().getX(),2) + Math.pow(y - follower.getPose().getY(),2));
        return (-0.0265 * Math.pow(dist,2)) + (11.5 * dist) + 580;
    }
    public void shooterVelocityTwo(double tps) {
        double error = shooter.getCorrectedVelocity() - tps;
        double power = pid.calculate(error,0);
        double clamped = clamp(power);
        shooter.set(clamped);
        shooter2.set(clamped);
        telemetry.addData("Error", error);
        telemetry.addData("Power", power);
        telemetry.addData("Clamped", clamped);
        telemetry.addData("Shooter Vel", shooter.getCorrectedVelocity());
    }
    public void autoAim(Follower follower) {
        double tR = getTurretRelToRobot();
        double rF = Math.toDegrees(follower.getHeading());
        double gF = Math.toDegrees(Math.atan2(y - follower.getPose().getY(), x - follower.getPose().getX()));
        double tt = rF - gF;
        double angleError = tt - tR;
        double power = pidTur.calculate(angleError, 0);
        double clamped = clamp2(power);
        turret.set(clamped);
        telemetry.addData("turret ticks", turret.getCurrentPosition());
        telemetry.addData("clamped", clamped);
        telemetry.addData("power", power);
        telemetry.addData("error", angleError);
        telemetry.addData("tR", tR);
        telemetry.addData("rF", rF);
        telemetry.addData("gF", gF);
        telemetry.addData("tt", tt);
    }
    public double clamp(double value) {
        return Math.max(-1, Math.min(1, value));
    }

    public double getTurretRelToRobot() {
        double current = turret.getCurrentPosition();
        double curFrac = current / (1484 * 2);
        double ans = (-curFrac * 360) % 360;
        return ans;
    }
    public double clamp2(double val) {
        return Math.max(-0.8, Math.min(0.8, val));
    }
    public void toggleServo() {
        if (servo.getPosition() == 0.34) {
            servo.setPosition(0.18);
        } else servo.setPosition(0.34);
    }
    public void detectTags(AprilTagProcessor ap) {
        List<AprilTagDetection> detections = ap.getDetections();
        if (detections != null && !detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                if (tag.ftcPose != null) {
                    telemetry.addData("X", tag.ftcPose.x);
                    telemetry.addData("Y", tag.ftcPose.y);
                    telemetry.addData("Z", tag.ftcPose.z);
                    telemetry.addData("Range", tag.ftcPose.range);
                    telemetry.addData("Bearing", tag.ftcPose.bearing);
                    telemetry.addData("Yaw", tag.ftcPose.yaw);
                    telemetry.addData("ID", tag.id);
                    telemetry.addData("Field X", tag.robotPose.getPosition().x);
                    telemetry.addData("Field Y", tag.robotPose.getPosition().y);
                    telemetry.addData("Field Z", tag.robotPose.getPosition().z);
                    lastTagDetected = tag;
                }
            }
        } else {
            lastTagDetected = null;
        }
    }
    public AprilTagDetection getTargetTag(int targetId1, int targetId2, int targetId3) {
        List<AprilTagDetection> detections = ap.getDetections();
        AprilTagDetection sol = null;
        for (AprilTagDetection tag : detections) {
            if (tag.id == targetId1 || tag.id == targetId2 || tag.id == targetId3) {
                sol = tag;
                break;
            }
        }
        return sol;
    }
    public boolean xInchRadius(int radius, AprilTagDetection target) {
        return target.ftcPose.range <= radius;
    }
    public void kalmanUpdate() {kalmanpose = (new Pose(kalmanX.getState(), kalmanY.getState(), kalmanH.getState()));}
    public void tagUpdate() {
        if (lastTagDetected != null) {
            if (lastTagDetected.robotPose != null) {
                apX = lastTagDetected.robotPose.getPosition().x;
                apY = lastTagDetected.robotPose.getPosition().y;
                //TODO: implement
                apH = lastTagDetected.robotPose.getOrientation().getYaw(AngleUnit.DEGREES) - 90;
            } else {
                apX = kalmanX.lastx;
                apY = kalmanY.lastx;
                apH = kalmanH.lastx;
            }
        } else {
            apX = kalmanX.lastx;
            apY = kalmanY.lastx;
            apH = kalmanH.lastx;
        }
    }
}

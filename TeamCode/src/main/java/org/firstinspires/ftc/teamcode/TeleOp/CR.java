package org.firstinspires.ftc.teamcode.TeleOp;
import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@Configurable
@TeleOp(name = "CR2/28")
public class CR extends LinearOpMode {
    public Motor rightFront;
    public Motor rightBack;
    public Motor leftFront;
    public Motor leftBack;
    public Motor shooter;
    public Motor shooter2;
    public Servo hoodServo;
    public Follower follower;
    public PIDController pidTur;
    public Motor transfer2;
    public DriveSubsystem drive;
    public PIDFController pid;
    public Motor transfer;
    public boolean lastServo = false;
    public double hoodPosition = 0.34;
    public Servo servo;
    public boolean lastUnload = false;
    public Timer timer;
    public CameraName cam;
    public VisionPortal vp;
    public AprilTagProcessor ap;
    private final Size CAMERA_RESOLUTION = new Size(640, 480);
    public AprilTagDetection lastTagDetected;
    public double apTag = 0;
    public static double alpha = 0.6;
    public TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    double lastValue = 0;
    public static double p = 0.035;
    public static double d = 0.003;
    public static double i = 0.01;
    public PIDController pidturn;
    public static double farDegrees = 0;
    public static double closeDegrees = 0;
    public Pose sp = new Pose(56,8, Math.toRadians(90));
    @Override
    public void runOpMode() throws InterruptedException {
        this.follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(sp);
        this.cam = hardwareMap.get(CameraName.class, "Webcam 1");
        this.pidturn = new PIDController(p, i, d);
        this.ap = new AprilTagProcessor.Builder()
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
        this.timer = new Timer();
        timer.resetTimer();
        leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        leftBack = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_435);
        rightBack = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_435);


        leftFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftFront.setRunMode(Motor.RunMode.RawPower);
        rightFront.setRunMode(Motor.RunMode.RawPower);
        leftBack.setRunMode(Motor.RunMode.RawPower);
        rightBack.setRunMode(Motor.RunMode.RawPower);

        leftFront.setInverted(true);
        rightFront.setInverted(true);
        leftBack.setInverted(true);
        rightBack.setInverted(true);


        leftFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        this.drive = new DriveSubsystem(
                leftFront,
                rightFront,
                leftBack,
                rightBack
        );
        transfer = new Motor(hardwareMap, "transfer", Motor.GoBILDA.RPM_435);
        transfer.setRunMode(Motor.RunMode.RawPower);
        transfer2 = new Motor(hardwareMap, "transfer2", Motor.GoBILDA.RPM_1620);
        transfer2.setRunMode(Motor.RunMode.RawPower);
        transfer2.setInverted(true);
        this.servo = hardwareMap.get(Servo.class, "tsservo");
        this.hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        this.pid = new PIDFController(0.004,0,0,0);
        this.shooter = new Motor(hardwareMap, "shooter", Motor.GoBILDA.RPM_1620);
        this.shooter2 = new Motor(hardwareMap, "shooter2", Motor.GoBILDA.RPM_1620);
        this.pidTur = new PIDController(0.008,0,0);
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
            follower.update();
            detectTags(ap);
            if (lastTagDetected != null) {
                panelsTelemetry.addData("Proximity", xInchRadius(15, lastTagDetected));
            }
            if (getTargetTag(21,22,23) != null) {
                int motifId = getTargetTag(21,22,23).id;
                if (motifId == 21) panelsTelemetry.addLine("GPP");
                else if (motifId == 22) panelsTelemetry.addLine("PGP");
                else panelsTelemetry.addLine("PPG");
            }
            double raw = 0;
            if (lastTagDetected != null) {
                raw = lastTagDetected.ftcPose.bearing;
                apTag = lastTagDetected.ftcPose.bearing;
                if (lastValue == 0) lastValue = apTag;
                else lastValue = getState(apTag, lastValue);
            } else lastValue = 0;
            panelsTelemetry.addData("ap filtered", lastValue);
            panelsTelemetry.addData("raw", raw);
            double strafee = gamepad1.left_stick_x;
            double forwardd = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double speed = 0.8;
            if (gamepad1.triangle) transfer.motor.setPower(-1);
            else if (gamepad1.circle) transfer.motor.setPower(1);
            else transfer.motor.setPower(0);
            if (gamepad1.square) transfer2.motor.setPower(-1);
            else if (gamepad1.cross) transfer2.motor.setPower(1);
            else transfer2.motor.setPower(0);
            if (gamepad1.dpad_up) servo.setPosition(0.18);
            else servo.setPosition(0.34);
            drive.controller.driveRobotCentric(strafee * speed, forwardd * speed, turn * speed);
            if (gamepad1.left_bumper) {
                //far zone
                hoodPosition = 0.11;
                shooterVelocityTwo(1600);
            }
            else if (gamepad1.right_bumper) {
                //close zone
                hoodPosition = 0.184;
                shooterVelocityTwo(1300);
            }
            else {
                hoodPosition = 0.185;
                shooter.set(0);
                shooter2.set(0);
            }
            //boolean shootmacro = gamepad1.options;
            //if (shootmacro && !lastUnload) unloadMag(timer);
            hoodServo.setPosition(hoodPosition);
            if (gamepad2.square) autoAim();
            if (gamepad2.cross) pinpointAim(farDegrees);
            if (gamepad2.triangle) pinpointAim(closeDegrees);
            panelsTelemetry.update(telemetry);
            telemetry.update();
            //lastUnload = shootmacro;
        }
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
    public double clamp(double value) {
        return Math.max(-1, Math.min(1, value));
    }
    public double clamp2(double val) {
        return Math.max(-0.8, Math.min(0.8, val));
    }
    public void detectTags(AprilTagProcessor ap) {
        List<AprilTagDetection> detections = ap.getDetections();
        if (detections != null && !detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                if (tag.ftcPose != null) {
                    panelsTelemetry.addData("X", tag.ftcPose.x);
                    panelsTelemetry.addData("Y", tag.ftcPose.y);
                    panelsTelemetry.addData("Z", tag.ftcPose.z);
                    panelsTelemetry.addData("Range", tag.ftcPose.range);
                    panelsTelemetry.addData("Bearing", tag.ftcPose.bearing);
                    panelsTelemetry.addData("Yaw", tag.ftcPose.yaw);
                    panelsTelemetry.addData("ID", tag.id);
                    panelsTelemetry.addData("Field X", tag.robotPose.getPosition().x);
                    panelsTelemetry.addData("Field Y", tag.robotPose.getPosition().y);
                    panelsTelemetry.addData("Field Z", tag.robotPose.getPosition().z);
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
    public double getState(double current, double last) {
        return last + alpha * (current - last);
    }
    public void autoAim(){
        if (lastTagDetected != null) {
            if (lastTagDetected.ftcPose != null) {
                double error = lastTagDetected.ftcPose.bearing;
                double p = pidturn.calculate(error,0);
                double set = clamp(p);
                drive.controller.driveRobotCentric(0,0,set);
                telemetry.addData("clamped", set);
                telemetry.addData("error", error);
                telemetry.addData("p", p);
            }
        }
    }
    public void pinpointAim(double degrees) {
        double h = Math.toDegrees(follower.getHeading());
        //maybe remove normalize to see if it turns the other way
        if (h < 0) {
            h = 360 - h;
        }
        double error = h - degrees;
        double p = pidturn.calculate(error,0);
        double clamped = clamp2(p);
        drive.controller.driveRobotCentric(0,0,clamped);
        telemetry.addData("Error (pinpoint)", error);
        telemetry.addData("Heading (pinpoint)", h);
        telemetry.addData("Raw Power (pinpoint)", p);
        telemetry.addData("Clamped (pinpoint)", clamped);
    }
}

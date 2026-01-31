package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.nio.charset.CharacterCodingException;
import java.util.concurrent.TimeUnit;

@Configurable
public class InDepSubsystem extends SubsystemBase {
    public HardwareRobot hardwareRobot;
    public DriveSubsystem drive;
    public LinearOpMode opMode;
    public PIDFController pid;
    public PIDController pidTurret;
    public InDepSubsystem(LinearOpMode opMode, HardwareMap hardwareMap) {
        this.hardwareRobot = new HardwareRobot(hardwareMap);
        this.drive = new DriveSubsystem(
          hardwareRobot.leftFront,
          hardwareRobot.rightFront,
          hardwareRobot.leftBack,
          hardwareRobot.rightBack
        );
        this.opMode = opMode;
        this.pid = new PIDFController(0.0027,0,0,0);
        this.pidTurret = new PIDController(0.008,0,0);
    }
    public void setShooterPower(double power){
        hardwareRobot.shooter.set(power);
    }
    public void setShooterVelocity(double tps) {
        double error = hardwareRobot.shooter.getCorrectedVelocity() - tps;
        double power = pid.calculate(error,0);
        double clamped = clamp(power);
        hardwareRobot.shooter.set(clamped);
        hardwareRobot.shooter2.set(clamped);
        opMode.telemetry.addData("Error", error);
        opMode.telemetry.addData("Power", power);
        opMode.telemetry.addData("Clamped", clamped);
        opMode.telemetry.addData("Shooter Vel", hardwareRobot.shooter.getCorrectedVelocity());
    }
    public void toggleServo() {
        if (hardwareRobot.transferServo.getPosition() == 0.75) {
            hardwareRobot.transferServo.setPosition(0.67);
        } else hardwareRobot.transferServo.setPosition(0.75);
    }
    public void setTurretPosition(double turretPos){
      double error = hardwareRobot.turret.getCurrentPosition() - turretPos;
      double power = pidTurret.calculate(error,0);
      double clamped = clamp2(power);
      hardwareRobot.turret.set(clamped);
      opMode.telemetry.addData("Error", error);
      opMode.telemetry.addData("Power", power);
      opMode.telemetry.addData("Clamped", clamped);
      opMode.telemetry.addData("Tur Pos", hardwareRobot.turret.getCurrentPosition());
      opMode.telemetry.update();
    }
    public void setTurretPower(double p) {
        hardwareRobot.turret.set(p);
    }
    public void setTransfer(double p) {
        hardwareRobot.transfer.motor.setPower(p);
    }
    public void hoodServo(double pos) {
        hardwareRobot.hoodServo.setPosition(pos);
    }

    //TODO: redo
    public void unloadMag(Timer opTimer) {
        int pathState = 1;
        opTimer.resetTimer();
        while (opMode.opModeIsActive()) {
            double strafe = opMode.gamepad1.left_stick_x;
            double forward = opMode.gamepad1.left_stick_y;
            double turn = opMode.gamepad1.right_stick_x;
            drive.driveRobotCentricPowers(strafe * 0.6, forward * 0.6, turn * 0.6);
            setTransfer(0.5);
            if (opTimer.getElapsedTimeSeconds() >= 3 && pathState == 1) {
                toggleServo();
                pathState++;
            }
            if (opTimer.getElapsedTimeSeconds() >= 3.5 && pathState == 2) {
                toggleServo();
                pathState++;
            }
            if (opTimer.getElapsedTimeSeconds() >= 5 && pathState == 3) {
                toggleServo();
                pathState++;
            }
            if (opTimer.getElapsedTimeSeconds() >= 5.5 && pathState == 4) {
                toggleServo();
                pathState++;
            }
            if (opTimer.getElapsedTimeSeconds() >= 8 && pathState == 5) {
                setTransfer(0);
                break;
            }
        }
    }
    public double clamp(double value) {
        return Math.max(-1, Math.min(1, value));
    }
    public double clamp2(double value) {
        return Math.max(-0.8, Math.min(0.8, value));
    }
    public double clamp3(double val) {
        return Math.max(45,Math.min(315, val));
    }
    public ElapsedTime time;
    public void resetUnloadMacro() {
        time = null;
    }

    public static int x;
    public static int y;
    public static Pose sp;
    public void initAutoAim(boolean blue, boolean far) {
        if (blue) {
            x = 10;
            y = 141;
        } else {
            x = 133;
            y = 141;
        }
        if (far && blue) sp = new Pose(33,135,Math.toRadians(270));
        if (!far && blue) sp = new Pose(56,8,Math.toRadians(90));
        if (far && !blue) sp = new Pose(111,135,Math.toRadians(270));
        if (!far && !blue) sp = new Pose(88,8.5,Math.toRadians(90));
    } //test
    public void autoAim(Follower follower) {
        double tR = getTurretRelToRobot();
        double rF = Math.toDegrees(follower.getHeading());
        double gF = Math.toDegrees(Math.atan2(y - follower.getPose().getY(), x - follower.getPose().getX()));
        double tt = rF - gF;
        double ttClamped = clamp3(tt);
        double angleError = tt - tR;
        double power = pidTurret.calculate(angleError, 0);
        double clamped = clamp2(power);
        hardwareRobot.turret.set(clamped);
        opMode.telemetry.addData("clamped", clamped);
        opMode.telemetry.addData("ttclamped", ttClamped);
        opMode.telemetry.addData("power", power);
        opMode.telemetry.addData("error", angleError);
        opMode.telemetry.addData("tR", tR);
        opMode.telemetry.addData("rF", rF);
        opMode.telemetry.addData("gF", gF);
        opMode.telemetry.addData("tt", tt);
        hoodServo(hoodAngle(x,y,follower));
        setShooterVelocity(shooterVelocity(x,y,follower));
    }
    public double hoodAngle(double x, double y, Follower follower) {
        double dist = Math.sqrt(Math.pow(x - follower.getPose().getX(),2) + Math.pow(y - follower.getPose().getY(), 2));
        return (-0.000551 * dist) + 0.128;
    }
    public double shooterVelocity(double x, double y, Follower follower) {
        double dist = Math.sqrt(Math.pow(x - follower.getPose().getX(),2) + Math.pow(y - follower.getPose().getY(),2));
        return (-0.0164 * Math.pow(dist,2)) + (10.3 * dist) + 540;
    }
    public double getTurretRelToRobot() {
        double current = hardwareRobot.turret.getCurrentPosition();
        double curFrac = current / (1484 * 2);
        double ans = (-curFrac * 360) % 360;
        return ans;
    }
}

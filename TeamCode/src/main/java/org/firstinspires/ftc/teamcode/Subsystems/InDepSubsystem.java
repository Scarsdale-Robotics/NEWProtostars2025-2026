package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.nio.charset.CharacterCodingException;
import java.util.concurrent.TimeUnit;

@Configurable
public class InDepSubsystem extends SubsystemBase {
    public HardwareRobot hardwareRobot;
    public DriveSubsystem drive;
    public LinearOpMode opMode;
    public static double KS;
    public static PIDController pid1 = new PIDController(0.0035,0.001,0);
    public InDepSubsystem(LinearOpMode opMode, HardwareMap hardwareMap) {
        this.hardwareRobot = new HardwareRobot(hardwareMap);
        this.drive = new DriveSubsystem(
          hardwareRobot.leftFront,
          hardwareRobot.rightFront,
          hardwareRobot.leftBack,
          hardwareRobot.rightBack
        );
        this.opMode = opMode;
    }
    public void setShooterPower(double power){
        hardwareRobot.shooterOne.set(-power);
        hardwareRobot.shooterTwo.set(power);
    }
    public void setShooterVel(double ticksPerSecond) {
        double error = hardwareRobot.shooterOne.getCorrectedVelocity() - ticksPerSecond;
        double power = pid1.calculate(error, 0);
        double clamped = clamp(power);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("P", pid1.getP());
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Power", power);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Error", error);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("velocity", hardwareRobot.shooterOne.getCorrectedVelocity());
        hardwareRobot.shooterOne.set(-clamped);
        hardwareRobot.shooterTwo.set(clamped);
    }
    public ElapsedTime time = null;
    double lastV = 0.0;
    //change both kP and initKP for velocity tuning
    public static double kP = 0.0035;
    public static double initKP = 0.0035;
    public void setShooterVelocity(double tps){
        if (!opMode.opModeIsActive()) return;
        if (time != null) {
            double dt = time.seconds();
            time.reset();
            PanelsTelemetry.INSTANCE.getTelemetry().addData("shooter dt (secs)", dt);
            double v = hardwareRobot.shooterOne.getCorrectedVelocity();
            double dv = v - lastV;
            lastV = v;
            PanelsTelemetry.INSTANCE.getTelemetry().addData("shooter v (tps)", v);
            double a = dv/dt;
            PanelsTelemetry.INSTANCE.getTelemetry().addData("shooter a (tpss)", a);
            double power = kP * (tps - v);
            PanelsTelemetry.INSTANCE.getTelemetry().addData("shooter power", power);
            PanelsTelemetry.INSTANCE.getTelemetry().addData("KP", kP);
            double clamped = clamp(power);
            PanelsTelemetry.INSTANCE.getTelemetry().addData("Clamped", clamped);
            hardwareRobot.shooterOne.set(-clamped);
            hardwareRobot.shooterTwo.set(clamped);
        } else {
            time = new ElapsedTime();
            kP = initKP;
            lastV = 0;
        }
    }
    public void setIntake(double p) {
        hardwareRobot.intakeTwo.set(p);
        hardwareRobot.intakeOne.set(p);
    }
    public void setFrontIn(double p) {
        hardwareRobot.intakeOne.set(p);
    }
    public void setSecondIn(double p) {
        hardwareRobot.intakeTwo.set(p);
    }

    public void toggleControlServo(double invertedPosition, double startingPosition) {
        if (hardwareRobot.intakeControl.getPosition() == 0) hardwareRobot.intakeControl.setPosition(0.31);
        else if (hardwareRobot.intakeControl.getPosition() == 0.31) hardwareRobot.intakeControl.setPosition(0);
    }
    //TODO: optimize
    public void unloadMag(Timer opTimer) {
        int pathState = 1;
        opTimer.resetTimer();
        while (opMode.opModeIsActive()) {
            double strafe = opMode.gamepad1.left_stick_x;
            double forward = opMode.gamepad1.left_stick_y;
            double turn = opMode.gamepad1.right_stick_x;
            drive.driveRobotCentricPowers(strafe * 0.6, forward * 0.6, turn * 0.6);
            setShooterVelocity(1810);
            if (opTimer.getElapsedTimeSeconds() >= 5 && pathState == 1) {
                toggleControlServo(0, 0.31);
                pathState++;
            }
            if (opTimer.getElapsedTimeSeconds() >= 5.5 && pathState == 2) {
                setSecondIn(0.7);
                pathState++;
            }
            if (opTimer.getElapsedTimeSeconds() >= 5.7 && pathState == 3) {
                toggleControlServo(0, 0.31);
                setSecondIn(0);
                pathState++;
            }
            if (opTimer.getElapsedTimeSeconds() >= 7.5 && pathState == 4) {
                toggleControlServo(0,0.31);
                pathState++;
            }
            if (opTimer.getElapsedTimeSeconds() >= 8 && pathState == 5) {
                setIntake(0.7);
                pathState++;
            }
            if (opTimer.getElapsedTimeSeconds() >= 9 && pathState == 6) {
                setSecondIn(0);
                toggleControlServo(0,0.31);
                break;
            }
        }
    }
    public double clamp(double value) {
        return Math.max(0, Math.min(1, value));
    }
    public void initControllers() {
        pid1.setTolerance(5);
        pid1.setSetPoint(0);
        KS = 0;
    }
    public void resetUnloadMacro() {
        time = null;
    }
    public PIDFController pidf = new PIDFController(0.01,0.0001,0.00001,0.015);
    public void autoAim(boolean blue, boolean red) {
        //TODO: approx polynomial curve for shooter vel
        //Use linear regression for hood angle
        //use trig for turret angle
        int x;
        int y;
        Pose sp;
        if (blue) {
            x = 0;
            y = 0;
            sp = new Pose(0,0,0);
            //tune
        } else {
            x = 0;
            y = 0;
            sp = new Pose(0, 0, 0);
            //tune
        }
        PIDController pid = new PIDController(0.01,0.001,0.0001);
        pid.setSetPoint(0);
        pid.setTolerance(1);
        while(opMode.opModeIsActive()) {
            double tR = 0; //possibly put pinpoint here. then, use rotation to determine angle
            double rF = hardwareRobot.pinpoint.getHeading(AngleUnit.DEGREES);
            double gF = 180 - Math.atan2(Math.abs(x - toFieldCoordinates(sp, hardwareRobot.pinpoint.getPosition()).getX()), Math.abs(y - toFieldCoordinates(sp, hardwareRobot.pinpoint.getPosition()).getY()));
            double tF = tR - rF;
            double angleError = gF - tF;
            double power = pid.calculate(angleError, 0);
            double clamped = clamp(power);
            //apply clamped power
            //part 2: hood angle
        }
    }
    public Pose toFieldCoordinates(Pose startPose, Pose2D currentPose) {
        return new Pose(0,0,0);
    }
    public double hoodAngle(double x, double y, Pose sp) {
        return 2 * Math.sqrt(Math.pow(x - toFieldCoordinates(sp, hardwareRobot.pinpoint.getPosition()).getX(),2) + Math.pow(y - toFieldCoordinates(sp, hardwareRobot.pinpoint.getPosition()).getY(), 2));
    }
}

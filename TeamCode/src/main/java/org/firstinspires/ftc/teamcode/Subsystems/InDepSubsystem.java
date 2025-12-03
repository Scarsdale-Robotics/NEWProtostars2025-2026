package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRobot;

import java.util.concurrent.TimeUnit;

@Configurable
public class InDepSubsystem extends SubsystemBase {
    public HardwareRobot hardwareRobot;
    public DriveSubsystem drive;
    public LinearOpMode opMode;
    public static double KS;
    public static PIDController pid1 = new PIDController(0.00001,0,0);
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
        pid1.setP(pid1.getP() + KS);
        double power = pid1.calculate(hardwareRobot.shooterOne.getCorrectedVelocity(), ticksPerSecond);
        double clamped = clamp(power);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("KS", KS);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("P", pid1.getP());
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Power", power);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Error", error);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("velocity", hardwareRobot.shooterOne.getCorrectedVelocity());
        hardwareRobot.shooterOne.set(-clamped);
        hardwareRobot.shooterTwo.set(clamped);
        KS += 0.0001;
    }
    public ElapsedTime time = null;
    double lastV = 0.0;
    public static double kP = 0.004, kD = 0.0, kS = 0.0;
    public double finalP = 0;
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
            double power = kP * (tps - v + kS) - kD * a;
            PanelsTelemetry.INSTANCE.getTelemetry().addData("shooter power", power);
            PanelsTelemetry.INSTANCE.getTelemetry().addData("KS", kS);
            PanelsTelemetry.INSTANCE.getTelemetry().addData("KP", kP);
            double clamped = clamp(power);
            PanelsTelemetry.INSTANCE.getTelemetry().addData("Clamped", clamped);
            hardwareRobot.shooterOne.set(-clamped);
            hardwareRobot.shooterTwo.set(clamped);
        } else {
            time = new ElapsedTime();
            kP = 0.004;
            kS = 0;
            lastV = 0;
            finalP = 0;
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
            setShooterVelocity(1760);
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
}

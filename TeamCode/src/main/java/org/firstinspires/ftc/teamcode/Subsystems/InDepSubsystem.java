package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
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
    public static PIDController pid1 = new PIDController(0.0008,0,0);
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
        double power = pid1.calculate(hardwareRobot.shooterOne.getCorrectedVelocity(), ticksPerSecond);
        double clamped = clamp(power);
        opMode.telemetry.addData("Power", power);
        opMode.telemetry.addData("Error", error);
        opMode.telemetry.addData("velocity", hardwareRobot.shooterOne.getCorrectedVelocity());
        hardwareRobot.shooterOne.set(-clamped);
        hardwareRobot.shooterTwo.set(clamped);
    }
    ElapsedTime time = null;
    double lastTicks = 0.0;
    double lastV = 0.0;
    public static double kP = 0.0008, kD = 0.0, kS = 0.0;
    public void setShooterVelocity(double tps){
        if (!opMode.opModeIsActive()) return;
        if (time != null) {
            double dt = time.seconds();
            time.reset();
            opMode.telemetry.addData("shooter dt (secs)", dt);

//            double ticks = hardwareRobot.shooterOne.getCurrentPosition();
//            opMode.telemetry.addData("shooter ticks (ticks)", ticks);
//            double dr = ticks - lastTicks;
//            lastTicks = ticks;
//            opMode.telemetry.addData("shooter dr (ticks)", dr);
//
//            double v = dr/dt;
            double v = hardwareRobot.shooterOne.getCorrectedVelocity();
            double dv = v - lastV;
            lastV = v;
            opMode.telemetry.addData("shooter v (tps)", v);
            double a = dv/dt;
            opMode.telemetry.addData("shooter a (tpss)", a);

            double power = kP * (tps - v + kS) - kD * a;
            opMode.telemetry.addData("shooter power", power);

            double clamped = clamp(power);
            hardwareRobot.shooterOne.set(-clamped);
            hardwareRobot.shooterTwo.set(clamped);
        } else {
            time = new ElapsedTime();
            lastTicks = hardwareRobot.shooterOne.getCurrentPosition();
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
            setIntake(0);
            setShooterPower(0.6);
            if (opTimer.getElapsedTimeSeconds() >= 2 && pathState == 1) {
                toggleControlServo(0, 0.31);
                pathState++;
            }
            if (opTimer.getElapsedTimeSeconds() >= 3 && pathState == 2) {
                setSecondIn(0.7);
                pathState++;
            }
            if (opTimer.getElapsedTimeSeconds() >= 7 && pathState == 3) {
                setSecondIn(0);
                toggleControlServo(0, 0.31);
                pathState++;
            }
            if (pathState == 4) break;
        }
    }
    public double clamp(double value) {
        return Math.max(-0.65, Math.min(0.65, value));
    }
}

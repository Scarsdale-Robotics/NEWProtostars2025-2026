package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareRobot;

public class InDepSubsystem extends SubsystemBase {
    public HardwareRobot hardwareRobot;
    public DriveSubsystem drive;
    public LinearOpMode opMode;
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
        hardwareRobot.shooterTwo.set(-1 * power);
        hardwareRobot.shooterOne.set(power);
    }
    //TODO: implement PID
    public void setShooterPowerVel(double p) {
        PDController pd = new PDController(0.1, 0);
        PDController pd2 = new PDController(0.1,0);
        pd.setTolerance(5);
        pd2.setTolerance(5);
        pd.setSetPoint(0);
        pd2.setSetPoint(0);
        while (!pd.atSetPoint() && !pd2.atSetPoint()) {
            double error1 = hardwareRobot.shooterOne.getCorrectedVelocity() - rpmToTicksPerSecond(convertToRPM(p));
            double error2 = hardwareRobot.shooterTwo.getCorrectedVelocity() - rpmToTicksPerSecond(convertToRPM(p));
            if (Math.abs(error1) < 7 && Math.abs(error2) < 7) break;
            double power1 = pd.calculate(error1, 6);
            double power2 = pd2.calculate(error2,6);
            double CP1 = clamp(power1);
            double CP2 = clamp(power2);
            hardwareRobot.shooterOne.set(CP1);
            hardwareRobot.shooterTwo.set(CP2);
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
        if (hardwareRobot.intakeControl.getPosition() == invertedPosition) hardwareRobot.intakeControl.setPosition(startingPosition);
        else hardwareRobot.intakeControl.setPosition(invertedPosition);
    }
    //TODO: optimize
    public void unloadMag(Timer opTimer) {
        int pathState = 1;
        opTimer.resetTimer();
        while (opMode.opModeIsActive()) {
            setIntake(0);
            setShooterPower(0.8);
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
    public double rpmToTicksPerSecond(double rpm) {
        final double TPR = 112.0;
        return (rpm / 60.0) * TPR;
    }
    public double convertToRPM(double p) {
        return 6000 * p;
    }
    public double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}

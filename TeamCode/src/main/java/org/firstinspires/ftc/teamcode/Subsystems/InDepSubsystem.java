package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareRobot;

public class InDepSubsystem extends SubsystemBase {
    public HardwareRobot hardwareRobot;
    public CVSubsystem cv;
    public DriveSubsystem drive;
    public LinearOpMode opMode;
    public InDepSubsystem(LinearOpMode opMode, HardwareMap hardwareMap) {
        this.hardwareRobot = new HardwareRobot(hardwareMap);
        this.cv = new CVSubsystem(hardwareRobot.cameraName, opMode, hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        this.drive = new DriveSubsystem(
          hardwareRobot.leftFront,
          hardwareRobot.rightFront,
          hardwareRobot.leftBack,
          hardwareRobot.rightBack
        );
        this.opMode = opMode;
    }
    public void setShooterPower(double power){
        hardwareRobot.shooterOne.set(power);
        hardwareRobot.shooterTwo.set(power);
    }
    public void setIntake(double p) {
        hardwareRobot.intakeOne.set(p);
        hardwareRobot.intakeTwo.set(p);
    }
    public void toggleControlServo(double invertedPosition, double startingPosition) {
        if (hardwareRobot.intakeControl.getPosition() == invertedPosition) hardwareRobot.intakeControl.setPosition(startingPosition);
        else hardwareRobot.intakeControl.setPosition(invertedPosition);
    }
    public void unloadMag() {
        toggleControlServo(0,1);
        opMode.sleep(1000);
        toggleControlServo(0,1);
        opMode.sleep(1000);
        toggleControlServo(0,1);
        opMode.sleep(1000);
        toggleControlServo(0,1);
    }
}

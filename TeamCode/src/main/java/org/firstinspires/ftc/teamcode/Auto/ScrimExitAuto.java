package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotSystem;
@Autonomous (name = "ExitAutoPosX")
public class ScrimExitAuto extends LinearOpMode {
    public RobotSystem robot;
    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        robot.hardwareRobot.initOdom();
        waitForStart();
        if (opModeIsActive()) {
            driveX(32);
        }
    }
    public void driveX(double targetX) {
        PIDController pid = new PIDController(0.4,0.04,0.002);
        pid.setSetPoint(0);
        pid.setTolerance(0.5);
        while (opModeIsActive() && !pid.atSetPoint()) {
            robot.hardwareRobot.pinpoint.update();
            double error = robot.hardwareRobot.pinpoint.getPosX(DistanceUnit.INCH) - targetX;
            if (Math.abs(error) <= 0.8) break;
            double power = pid.calculate(error, 0);
            double clamped = clamp(power);
            robot.drive.driveRobotCentricPowers(clamped,0,0);
            telemetry.addData("Power", power);
            telemetry.addData("Error", error);
            telemetry.addData("PosX", robot.hardwareRobot.pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("TargetX", targetX);
            telemetry.update();
        }
    }
    public double clamp(double value) {
        return Math.max(-0.55, Math.min(0.55, value));
    }
}

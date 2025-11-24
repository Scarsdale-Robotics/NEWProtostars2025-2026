package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotSystem;
//TODO: finish tuning and transfer coefficients to teleop
@Autonomous (name = "TurnAuton")
public class TurnAutoTest extends LinearOpMode {
    public RobotSystem robot;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        robot.hardwareRobot.initOdom();
        waitForStart();
        if (opModeIsActive()) {
            turn(90);
        }
    }

    public void turn(double targetHeading) {
        PIDController pid = new PIDController(0.02, 0.03, 0.002);
        pid.setSetPoint(targetHeading);
        pid.setTolerance(1); // 1-degree tolerance

        while (opModeIsActive()) {
            robot.hardwareRobot.pinpoint.update();

            double currentHeading = robot.hardwareRobot.pinpoint.getHeading(AngleUnit.DEGREES);

            // Normalize so PID doesn't jump from +179 to -181
            currentHeading = normalizeAngle(currentHeading);

            // PID computes: error = normalize(target - current)
            double power = pid.calculate(currentHeading);

            // Stop when close enough
            if (pid.atSetPoint()) break;

            double clamped = clamp(power);

            robot.drive.driveRobotCentricPowers(0, 0, clamped);

            telemetry.addData("Power", power);
            telemetry.addData("Error", pid.getPositionError());
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("TargetHeading", targetHeading);
            telemetry.update();
            robot.hardwareRobot.pinpoint.update();
        }
    }

    public double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle <= -180) angle += 360;
        if (angle > 180) angle -= 360;
        return angle;
    }

    public double clamp(double value) {
        return Math.max(-0.6, Math.min(0.6, value));
    }
}

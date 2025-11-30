package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotSystem;
//TODO: finish tuning and transfer coefficients to teleop
@Autonomous (name = "TurnAuton11/29")
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
        PIDController pid = new PIDController(0.06, 0.05, 0.002);
        pid.setSetPoint(targetHeading);
        pid.setTolerance(1); // 1-degree tolerance

        while (opModeIsActive() && !pid.atSetPoint()) {
            robot.hardwareRobot.pinpoint.update();

            double currentHeading = robot.hardwareRobot.pinpoint.getHeading(AngleUnit.DEGREES);

            double corrected = normalizeAngle(currentHeading);
            double error = corrected - targetHeading;

            double power = pid.calculate(corrected, targetHeading);

            if (Math.abs(error) <= 2) break;

            double clamped = clamp(power);

            robot.drive.driveRobotCentricPowers(0, 0, clamped);

            telemetry.addData("Power", power);
            telemetry.addData("Error", error);
            telemetry.addData("Corrected H", corrected);
            telemetry.addData("Raw H", currentHeading);
            telemetry.addData("PPX", robot.hardwareRobot.pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("PPY", robot.hardwareRobot.pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("TargetHeading", targetHeading);
            telemetry.update();
        }
    }

    public double normalizeAngle(double angle) {
        if (angle <= 0) return Math.abs(angle);
        else if (angle > 0) return (180 - angle) + 180;
        return 0;
    }

    public double clamp(double value) {
        return Math.max(-0.6, Math.min(0.6, value));
    }
}

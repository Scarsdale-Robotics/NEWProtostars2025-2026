package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.RobotSystem;
@Autonomous (name = "ShooterTest")
public class ShooterTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Motor shooterOne = new Motor(hardwareMap, "shooterOne", Motor.GoBILDA.RPM_1620);
        Motor shooterTwo = new Motor(hardwareMap, "shooterTwo", Motor.GoBILDA.RPM_1620);

        shooterOne.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterTwo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterOne.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterTwo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterOne.setRunMode(Motor.RunMode.RawPower);
        shooterTwo.setRunMode(Motor.RunMode.RawPower);

        shooterOne.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterTwo.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            shooterOne.motor.setPower(1.0);
            shooterTwo.motor.setPower(-1.0);
        }
    }
}

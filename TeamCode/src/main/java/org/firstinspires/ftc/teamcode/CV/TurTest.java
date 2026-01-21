package org.firstinspires.ftc.teamcode.CV;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp (name = "TurTest")
public class TurTest extends LinearOpMode {
    public Motor turret;
    public PIDController pidTur;
    public static double kP = 0.01;
    public static double kI = 0.001;
    public static double kD = 0.00001;
    @Override
    public void runOpMode() throws InterruptedException {
        this.pidTur = new PIDController(0.003,0,0);
        turret = new Motor(hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
        //transfer = new Motor(hardwareMap, "transfer", Motor.GoBILDA.RPM_1620);
        turret.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //transfer.motor.setMode(DcMoto
        turret.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //transfer.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setRunMode(Motor.RunMode.RawPower);
        //transfer.setRunMode(Motor.RunMode.RawPower);
        turret.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //transfer.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAK
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        if (opModeIsActive()) {
            turretPID(400);
            turretPID(0);
        }
    }
    public void turretPID(double turretPos) {
        while (opModeIsActive()) {
            double error = turret.getCurrentPosition() - turretPos;
            double power = pidTur.calculate(error,0);
            double clamped = clamp(power);
            turret.set(clamped);
            telemetry.addData("Error", error);
            telemetry.addData("Power", power);
            telemetry.addData("Clamped", clamped);
            telemetry.addData("Tur Pos", turret.getCurrentPosition());
            telemetry.update();
            if (Math.abs(error) < 1) break;
        }
    }
    public double clamp(double value) {
        return Math.max(-0.8, Math.min(0.8, value));
    }
}

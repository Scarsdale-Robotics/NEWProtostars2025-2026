package org.firstinspires.ftc.teamcode.CV;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;

@TeleOp (name = "AATest1/21")
public class AutoAimTest extends LinearOpMode {
  public Motor turret;
  public PIDController pidTur;
  public static double tps = -1400;
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
    while (opModeIsActive()) {

    }
  }
  public double clamp(double value) {
    return Math.max(-0.8, Math.min(0.8, value));
  }
  public double getTurretRelToRobot() {
    double current = turret.getCurrentPosition();
    double curFrac = current / (1484 * 2);
    double ans = curFrac * 360;
    return ans;
  }
}

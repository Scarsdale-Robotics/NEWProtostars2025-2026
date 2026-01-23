package org.firstinspires.ftc.teamcode.CV;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp (name = "AATest1/21")
public class AutoAimTest extends LinearOpMode {
  public Motor turret;
  public PIDController pidTur;
  public static double tps = -1400;
  public boolean lastAlign = false;
  public GoBildaPinpointDriver pinpoint;
  public static double x = 13;
  public static double y = 135;
  public Follower follower;
  public FollowerConstants constants;
  public Pose startPose = new Pose(56,8,Math.toRadians(90));
  @Override
  public void runOpMode() throws InterruptedException {
      follower = Constants.createFollower(hardwareMap);
      follower.setStartingPose(startPose);
      this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
      this.pidTur = new PIDController(0.004,0,0);
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
          follower.update();
          telemetry.addData("Follower X", follower.getPose().getX());
          telemetry.addData("Follower Y", follower.getPose().getY());
          telemetry.addData("Follower H", Math.toDegrees(follower.getPose().getHeading()));
          telemetry.addData("tr", getTurretRelToRobot());
          telemetry.addData("rf", Math.toDegrees(follower.getHeading()));
          telemetry.addData("gf", Math.toDegrees(Math.atan2(y - follower.getPose().getY(), x - follower.getPose().getX())));
          telemetry.addData("tt", Math.toDegrees(Math.atan2(y - follower.getPose().getY(), x - follower.getPose().getX())) - Math.toDegrees(follower.getHeading()));
          double tR = getTurretRelToRobot();
          double rF = Math.toDegrees(follower.getHeading());
          double gF = Math.toDegrees(Math.atan2(y - follower.getPose().getY(), x - follower.getPose().getX()));
          double tt = gF - rF;
          double angleError = tt - tR;
          autoAim(follower);
          telemetry.update();
      }
  }
    public double hoodAngle(double x, double y, Follower follower) {
        return 2 * Math.sqrt(Math.pow(x - follower.getPose().getX(),2) + Math.pow(y - follower.getPose().getY(), 2));
    }
    public double shooterVelocity(double x, double y, Follower follower) {
        double dist = Math.sqrt(Math.pow(x - follower.getPose().getX(),2) + Math.pow(y - follower.getPose().getY(),2));
        return 2 * Math.pow(dist,2) + 6 * dist - 5;
    }
    public void autoAim(Follower follower) {
          double tR = getTurretRelToRobot();
          double rF = Math.toDegrees(follower.getHeading());
          double gF = Math.toDegrees(Math.atan2(y - follower.getPose().getY(), x - follower.getPose().getX()));
          double tt = rF - gF;
          double angleError = tt - tR;
          double power = pidTur.calculate(angleError, 0);
          double clamped = clamp(power);
          turret.set(clamped);
          telemetry.addData("clamped", clamped);
          telemetry.addData("power", power);
          telemetry.addData("error", angleError);
          telemetry.addData("tR", tR);
        telemetry.addData("rF", rF);
        telemetry.addData("gF", gF);
        telemetry.addData("tt", tt);
        telemetry.update();
    }
  public double clamp(double value) {
    return Math.max(-1, Math.min(1, value));
  }
  public double getTurretRelToRobot() {
    double current = turret.getCurrentPosition();
    double curFrac = current / (1484 * 2);
    double ans = -curFrac * 360;
    return ans;
  }
}

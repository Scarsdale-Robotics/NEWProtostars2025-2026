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
  public GoBildaPinpointDriver pinpoint;
  public static double x = 13;
  public static double y = 135;
  public Follower follower;
  public FollowerConstants constants;
  public Pose startPose = new Pose(0,0,0);
  @Override
  public void runOpMode() throws InterruptedException {
      follower = Constants.createFollower(hardwareMap);
      follower.setStartingPose(startPose);
      this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
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
          follower.update();
          autoAim(follower);
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
        double rF = pinpoint.getHeading(AngleUnit.DEGREES);
        double gF = Math.toDegrees(Math.atan2(Math.abs(x - follower.getPose().getX()), Math.abs(y - follower.getPose().getY())));
        double tF = tR - rF;
        double angleError = 90 - gF - tF;
        double power = pidTur.calculate(angleError, 0);
        double clamped = clamp(power);
        turret.set(clamped);
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

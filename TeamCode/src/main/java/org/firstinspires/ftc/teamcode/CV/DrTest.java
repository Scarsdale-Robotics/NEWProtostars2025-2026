package org.firstinspires.ftc.teamcode.CV;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Drive Test 1/27")
public class DrTest extends LinearOpMode {
    public Timer time;
    public static double kP = 0.0027;
    public static double kP2 = 0.001;
    public static double kI2 = 0.001;
    public static double kD2 = 0.001;
    public static double initKP = 0.0027;
    public double lastV = 0;
    public static double kF = 0;
    public static double hoodpos = 0;
    public Motor rightFront;
    public Motor rightBack;
    public Motor leftFront;
    public Motor leftBack;
    public Motor shooter;
    public Motor shooter2;
    public boolean lastTurret = false;
    public Servo hoodServo;
    public DriveSubsystem drive;
    public PIDFController pid;
    public GoBildaPinpointDriver pp;
    public boolean lastServo = false;
    public PIDController pidTur;
    public Motor turret;
    //public Motor transfer;
    public static boolean lb = false;
    public static boolean rb = false;
    public static boolean rf = false;
    public static boolean lf = false;
    public Follower follower;
    public FollowerConstants constants;
    public static int x = 133;
    public static int y = 141;
    public Pose startPose = new Pose(88,8.5,Math.toRadians(90));
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        leftBack = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_435);
        rightBack = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_435);
        turret = new Motor(hardwareMap, "turret", Motor.GoBILDA.RPM_312);


        leftFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        leftFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftFront.setRunMode(Motor.RunMode.RawPower);
        rightFront.setRunMode(Motor.RunMode.RawPower);
        leftBack.setRunMode(Motor.RunMode.RawPower);
        rightBack.setRunMode(Motor.RunMode.RawPower);
        turret.setRunMode(Motor.RunMode.RawPower);

        leftFront.setInverted(true);
        rightFront.setInverted(true);
        leftBack.setInverted(true);
        rightBack.setInverted(true);


        leftFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        this.drive = new DriveSubsystem(
                leftFront,
                rightFront,
                leftBack,
                rightBack
        );

        this.hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        this.pid = new PIDFController(0.0027,0,0,0);
        this.shooter = new Motor(hardwareMap, "shooter", Motor.GoBILDA.RPM_1620);
        this.shooter2 = new Motor(hardwareMap, "shooter2", Motor.GoBILDA.RPM_1620);
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
        shooter.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //transfer.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //transfer.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setInverted(true);
        shooter2.setInverted(true);

        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter2.setRunMode(Motor.RunMode.RawPower);
        //transfer.setRunMode(Motor.RunMode.RawPower);

        shooter.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //transfer.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double speed = 0.5;
            telemetry.addData("strafe", strafe);
            telemetry.addData("forward", forward);
            telemetry.addData("turn", turn);
            telemetry.addData("LBPower", leftBack.get());
            telemetry.addData("LFPower", leftFront.get());
            telemetry.addData("RBPower", rightBack.get());
            telemetry.addData("RFPower", rightFront.get());
            telemetry.addData("TR", getTurretRelToRobot());
            telemetry.addData("Follower X", follower.getPose().getX());
            telemetry.addData("Follower Y", follower.getPose().getY());
            telemetry.addData("Follower H", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("distance", Math.sqrt(Math.pow(x - follower.getPose().getX(),2) + Math.pow(y - follower.getPose().getY(),2)));
            drive.controller.driveRobotCentric(strafe * speed, forward * speed, turn * speed);
            autoAim(follower);
            shooterVelocityTwo(shooterVelocity(x,y,follower));
            hoodServo.setPosition(hoodAngle(x,y,follower));
            telemetry.update();
        }
    }
    public double hoodAngle(double x, double y, Follower follower) {
        double dist = Math.sqrt(Math.pow(x - follower.getPose().getX(),2) + Math.pow(y - follower.getPose().getY(), 2));
        return (-0.000551 * dist) + 0.128;
    }
    public double shooterVelocity(double x, double y, Follower follower) {
        double dist = Math.sqrt(Math.pow(x - follower.getPose().getX(),2) + Math.pow(y - follower.getPose().getY(),2));
        return (-0.0164 * Math.pow(dist,2)) + (10.3 * dist) + 540;
    }
    public void shooterVelocityTwo(double tps) {
        double error = shooter.getCorrectedVelocity() - tps;
        double power = pid.calculate(error,0);
        double clamped = clamp(power);
        shooter.set(clamped);
        shooter2.set(clamped);
        telemetry.addData("Error", error);
        telemetry.addData("Power", power);
        telemetry.addData("Clamped", clamped);
        telemetry.addData("Shooter Vel", shooter.getCorrectedVelocity());
    }
    public void autoAim(Follower follower) {
        if (getTurretRelToRobot() == 179) {
            //switchback condition to true
        }
        else if (getTurretRelToRobot() == -179) {
            //switchback condition to true
        }
        double tR = getTurretRelToRobot();
        double rF = Math.toDegrees(follower.getHeading());
        double gF = Math.toDegrees(Math.atan2(y - follower.getPose().getY(), x - follower.getPose().getX()));
        double tt = rF - gF;
        double angleError = tt - tR;
        double power = pidTur.calculate(angleError, 0);
        double clamped = clamp2(power);
        turret.set(clamped);
        telemetry.addData("turret ticks", turret.getCurrentPosition());
        telemetry.addData("clamped", clamped);
        telemetry.addData("power", power);
        telemetry.addData("error", angleError);
        telemetry.addData("tR", tR);
        telemetry.addData("rF", rF);
        telemetry.addData("gF", gF);
        telemetry.addData("tt", tt);
    }
    public double clamp(double value) {
        return Math.max(-1, Math.min(1, value));
    }

    public double getTurretRelToRobot() {
        double current = turret.getCurrentPosition();
        double curFrac = current / (1484 * 2);
        double ans = (-curFrac * 360) % 360;
        return ans;
    }
    public double cumulativeTR() {
        double current = turret.getCurrentPosition();
        double curFrace = current / (1484 * 2);
        double ans = (-curFrace * 360);
        return ans;
    }
    public double clamp2(double val) {
        return Math.max(-0.8, Math.min(0.8, val));
    }
    public double clamp3(double val) {
        return Math.max(45,Math.min(315, val));
    }
}

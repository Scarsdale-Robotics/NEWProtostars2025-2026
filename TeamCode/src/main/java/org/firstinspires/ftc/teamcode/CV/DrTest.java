package org.firstinspires.ftc.teamcode.CV;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "General Tele Test 1/31")
public class DrTest extends LinearOpMode {
    public Motor rightFront;
    public Motor rightBack;
    public Motor leftFront;
    public Motor leftBack;
    public Motor shooter;
    public Motor shooter2;
    public Servo hoodServo;
    public DriveSubsystem drive;
    public PIDFController pid;
    public PIDController pidTur;
    public Motor turret;
    public Follower follower;
    public FollowerConstants constants;
    public Motor transfer;
    public boolean lastServo = false;
    public Servo servo;
    public static int x = 5;
    public static int y = 141;
    public Pose startPose = new Pose(56,8,Math.toRadians(90));
    public static double ticks;
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
        transfer = new Motor(hardwareMap, "transfer", Motor.GoBILDA.RPM_435);
        transfer.setRunMode(Motor.RunMode.RawPower);
        this.servo = hardwareMap.get(Servo.class, "tsservo");
        this.hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        this.pid = new PIDFController(0.0027,0,0,0);
        this.shooter = new Motor(hardwareMap, "shooter", Motor.GoBILDA.RPM_1620);
        this.shooter2 = new Motor(hardwareMap, "shooter2", Motor.GoBILDA.RPM_1620);
        this.pidTur = new PIDController(0.008,0,0);
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
            double strafee = gamepad1.left_stick_x;
            double forwardd = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double speed = 0.5;
            if (gamepad1.triangle) transfer.motor.setPower(-1);
            else if (gamepad1.circle) transfer.motor.setPower(1);
            else transfer.motor.setPower(0);
            if (gamepad1.dpad_up && !lastServo) toggleServo();
            telemetry.addData("Follower X", follower.getPose().getX());
            telemetry.addData("Follower Y", follower.getPose().getY());
            telemetry.addData("Follower H", Math.toDegrees(follower.getHeading()));
            telemetry.addData("distance", Math.sqrt(Math.pow(x - follower.getPose().getX(),2) + Math.pow(y - follower.getPose().getY(),2)));
            drive.controller.driveRobotCentric(strafee * speed, forwardd * speed, turn * speed);
            autoAim(follower);
            //shooterVelocityTwo(shooterVelocity(x,y,follower));
            shooterVelocityTwo(ticks);
            hoodServo.setPosition(hoodAngle(x,y,follower));
            telemetry.update();
            lastServo = gamepad1.dpad_up;
        }
    }
    public double hoodAngle(double x, double y, Follower follower) {
        double dist = Math.sqrt(Math.pow(x - follower.getPose().getX(),2) + Math.pow(y - follower.getPose().getY(), 2));
        return (-0.000687 * dist) + 0.207;
    }
    public double shooterVelocity(double x, double y, Follower follower) {
        double dist = Math.sqrt(Math.pow(x - follower.getPose().getX(),2) + Math.pow(y - follower.getPose().getY(),2));
        return (-0.0265 * Math.pow(dist,2)) + (11.5 * dist) + 580;
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
    public double clamp2(double val) {
        return Math.max(-0.8, Math.min(0.8, val));
    }
    public void toggleServo() {
        if (servo.getPosition() == 0.34) {
            servo.setPosition(0.18);
        } else servo.setPosition(0.34);
    }
}

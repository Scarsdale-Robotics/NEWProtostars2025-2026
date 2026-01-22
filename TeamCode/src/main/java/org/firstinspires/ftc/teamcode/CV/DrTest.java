package org.firstinspires.ftc.teamcode.CV;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
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

@Configurable
@TeleOp(name = "Drive + Shooter 1/14")
public class DrTest extends LinearOpMode {
    public Timer time;
    public static double kP = 0.0027;
    public static double kP2 = 0.001;
    public static double kI2 = 0.001;
    public static double kD2 = 0.001;
    public static double initKP = 0.0027;
    public double lastV = 0;
    public static double kF = 0;
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
    @Override
    public void runOpMode() throws InterruptedException {
        this.pidTur = new PIDController(kP2,kI2,kD2);
        this.pid = new PIDFController(kP, 0, 0,kF);
        this.time = null;
        this.hoodServo = hardwareMap.get(Servo.class, "servo");
        this.shooter = new Motor(hardwareMap, "shooter", Motor.GoBILDA.RPM_1620);
        this.shooter2 = new Motor(hardwareMap, "shooter2", Motor.GoBILDA.RPM_1620);
        this.pp = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        pp.setOffsets(7,1.5, DistanceUnit.INCH);
        pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pp.resetPosAndIMU();
        leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        leftBack = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_435);
        rightBack = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_435);
        turret = new Motor(hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
        //transfer = new Motor(hardwareMap, "transfer", Motor.GoBILDA.RPM_1620);


        leftFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //transfer.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //transfer.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setRunMode(Motor.RunMode.RawPower);
        rightFront.setRunMode(Motor.RunMode.RawPower);
        leftBack.setRunMode(Motor.RunMode.RawPower);
        rightBack.setRunMode(Motor.RunMode.RawPower);
        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter2.setRunMode(Motor.RunMode.RawPower);
        turret.setRunMode(Motor.RunMode.RawPower);
        //transfer.setRunMode(Motor.RunMode.RawPower);

        leftFront.setInverted(true);
        rightFront.setInverted(false);
        leftBack.setInverted(true);
        rightBack.setInverted(false);

        leftFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //transfer.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);



        this.drive = new DriveSubsystem(
                leftFront,
                rightFront,
                leftBack,
                rightBack
        );
        waitForStart();
        while (opModeIsActive()) {
            pp.update();
            boolean servo = gamepad1.circle;
            if (servo && !lastServo) {
                if (hoodServo.getPosition() == 0) {
                    hoodServo.setPosition(-0.3);
                } else hoodServo.setPosition(0);
            }
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double speed = 0.5;
            if (gamepad1.dpad_up) setShooterVelocity(1200);
            else if (gamepad1.dpad_down) {
                shooterVelocityTwo(-1400);
            }
            else {
                shooter.set(0);
                shooter2.set(0);
            }
            //transfer.set(-gamepad1.right_trigger);
            telemetry.addData("X", pp.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y", pp.getPosY(DistanceUnit.INCH));
            telemetry.addData("H", pp.getHeading(AngleUnit.DEGREES));
            telemetry.addData("dpad up", gamepad1.dpad_up);
            telemetry.addData("dpad down", gamepad1.dpad_down);
            telemetry.addData("dpad right", gamepad1.dpad_right);
            telemetry.addData("dpad left", gamepad1.dpad_left);
            telemetry.addData("Hood", hoodServo.getPosition());
            telemetry.addData("TurretPosition", turret.getCurrentPosition());
            telemetry.addData("TR", getTurretRelToRobot());
            telemetry.update();
            drive.driveRobotCentricPowers(strafe * speed, forward * speed, turn * speed);
            lastServo = servo;
            lastTurret = gamepad1.right_bumper;
        }
    }
    public void setShooterVelocity(double tps){
        if (!opModeIsActive()) return;
        if (time != null) {
            double dt = time.getElapsedTimeSeconds();
            time.resetTimer();
            telemetry.addData("shooter dt (secs)", dt);
            double v = shooter.getCorrectedVelocity();
            double dv = v - lastV;
            lastV = v;
            telemetry.addData("shooter v (tps)", v);
            double a = dv/dt;
            PanelsTelemetry.INSTANCE.getTelemetry().addData("shooter a (tpss)", a);
            double power = kP * (tps - v);
            telemetry.addData("shooter power", power);
            telemetry.addData("KP", kP);
            double clamped = clamp(power);
            telemetry.addData("Clamped", clamped);
            shooter.set(-clamped);
            shooter2.set(-clamped);
        } else {
            time = new Timer();
            kP = initKP;
            lastV = 0;
        }
    }
    public void turretPID(double tps) {
        while (opModeIsActive()) {
            double error = turret.getCorrectedVelocity() - tps;
            double power = pidTur.calculate(error,0);
            double clamped = clamp(power);
            turret.set(clamped);
            telemetry.addData("Error", error);
            telemetry.addData("Power", power);
            telemetry.addData("Clamped", clamped);
            telemetry.addData("Tur Pos", turret.getCurrentPosition());
        }
    }
    public double getTurretRelToRobot() {
        double current = turret.getCurrentPosition();
        double curFrac = current / (1484 * 2);
        double ans = curFrac * 360;
        return ans;
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
    public double clamp(double value) {
        return Math.max(-1, Math.min(1, value));
    }
}

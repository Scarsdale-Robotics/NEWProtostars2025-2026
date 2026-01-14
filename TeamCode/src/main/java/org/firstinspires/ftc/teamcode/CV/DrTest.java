package org.firstinspires.ftc.teamcode.CV;

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
    public static double kP = 0.0005;
    public static double initKP = 0.0005;
    public double lastV = 0;
    public double kD = 0.000001;
    public Motor rightFront;
    public Motor rightBack;
    public Motor leftFront;
    public Motor leftBack;
    public Motor shooter;
    public Servo hoodServo;
    public DriveSubsystem drive;
    public GoBildaPinpointDriver pp;
    //public Motor transfer;
    @Override
    public void runOpMode() throws InterruptedException {
        this.time = null;
        this.hoodServo = hardwareMap.get(Servo.class, "servo");
        this.shooter = new Motor(hardwareMap, "shooter", Motor.GoBILDA.RPM_1620);
        this.pp = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        pp.setOffsets(7,1.5, DistanceUnit.INCH);
        pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pp.resetPosAndIMU();
        leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        leftBack = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_435);
        rightBack = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_435);
        //transfer = new Motor(hardwareMap, "transfer", Motor.GoBILDA.RPM_1620);


        leftFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //transfer.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //transfer.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setRunMode(Motor.RunMode.RawPower);
        rightFront.setRunMode(Motor.RunMode.RawPower);
        leftBack.setRunMode(Motor.RunMode.RawPower);
        rightBack.setRunMode(Motor.RunMode.RawPower);
        shooter.setRunMode(Motor.RunMode.RawPower);
        //transfer.setRunMode(Motor.RunMode.RawPower);

        leftFront.setInverted(false);
        rightFront.setInverted(true);
        leftBack.setInverted(true);
        rightBack.setInverted(false);

        leftFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //transfer.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
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
            if (gamepad1.circle) {
                hoodServo.setPosition(hoodServo.getPosition() + 0.001);
            }
            else if (gamepad1.triangle) {
                hoodServo.setPosition(hoodServo.getPosition() - 0.001);
            }
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double speed = 0.5;
            if (gamepad1.dpad_up) setShooterVelocity(1400);
            if (gamepad1.dpad_down) setShooterVelocity(1100);
            if (gamepad1.dpad_left) shooter.set(0.9);
            if (gamepad1.dpad_down) shooter.set(0.8);
            else setShooterVelocity(0);
            //transfer.set(-gamepad1.right_trigger);
            telemetry.addData("X", pp.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y", pp.getPosY(DistanceUnit.INCH));
            telemetry.addData("H", pp.getHeading(AngleUnit.DEGREES));
            telemetry.addData("dpad up", gamepad1.dpad_up);
            telemetry.addData("dpad down", gamepad1.dpad_down);
            telemetry.addData("dpad right", gamepad1.dpad_right);
            telemetry.addData("dpad left", gamepad1.dpad_left);
            telemetry.update();
            drive.driveRobotCentricPowers(strafe * speed, forward * speed, turn * speed);
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
            shooter.set(clamped);
        } else {
            time = new Timer();
            kP = initKP;
            lastV = 0;
        }
    }
    public double clamp(double value) {
        return Math.max(0, Math.min(1, value));
    }
}

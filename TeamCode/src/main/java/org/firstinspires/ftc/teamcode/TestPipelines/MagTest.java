package org.firstinspires.ftc.teamcode.TestPipelines;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp (name = "Mag Test 1/14")
public class MagTest extends LinearOpMode {
    public Motor transfer;
    public Timer time;
    public Servo servo;
    public boolean lastUnload = false;
    public boolean lastServo = false;
    @Override
    public void runOpMode() throws InterruptedException {
        transfer = new Motor(hardwareMap, "transfer", Motor.GoBILDA.RPM_1620);
        transfer.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setRunMode(Motor.RunMode.RawPower);
        transfer.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.servo = hardwareMap.get(Servo.class, "servo");
        this.time = new Timer();
        waitForStart();
        while (opModeIsActive()) {
            boolean transferPressed = gamepad1.triangle;
            if (transferPressed) transfer.set(0.6);
            else if (gamepad1.circle) transfer.set(-0.6);
            else transfer.set(0);
            boolean serv = gamepad1.square;
            if (serv && !lastServo) toggleServo();
            boolean macro = gamepad1.options;
            if (macro && !lastUnload) unloadMag(time);
            lastServo = serv;
            lastUnload = macro;
            telemetry.addData("Servo Last", lastServo);
            telemetry.addData("servo pos", servo.getPosition());
            telemetry.addData("servo pressed", serv);
            telemetry.addData("square (servo)", gamepad1.square);
            telemetry.addData("Macro", macro);
            telemetry.addData("options (macro)", gamepad1.options);
            telemetry.addData("macro Last", lastUnload);
            telemetry.addData("transfer pressed", transferPressed);
            telemetry.addData("transfer power", transfer.get());
            telemetry.addData("triangle (transfer)", gamepad1.triangle);
            telemetry.addData("circle (reverse transfer)", gamepad1.circle);
            telemetry.addData("joystick right y", gamepad1.right_stick_y);
            telemetry.update();
        }
    }
    public void unloadMag(Timer opTimer) {
        int pathState = 1;
        opTimer.resetTimer();
        while (opModeIsActive()) {
            transfer.set(-1);
            if (opTimer.getElapsedTimeSeconds() >= 3 && pathState == 1) {
                toggleServo();
                pathState++;
            }
            if (opTimer.getElapsedTimeSeconds() >= 3.5 && pathState == 2) {
                toggleServo();
                pathState++;
            }
            if (opTimer.getElapsedTimeSeconds() >= 5 && pathState == 3) {
                toggleServo();
                pathState++;
            }
            if (opTimer.getElapsedTimeSeconds() >= 5.5 && pathState == 4) {
                toggleServo();
                pathState++;
            }
            if (opTimer.getElapsedTimeSeconds() >= 8 && pathState == 5) {
                transfer.set(0);
                break;
            }
        }
    }
    public void toggleServo() {
        if (servo.getPosition() == 0.38) {
            servo.setPosition(0.57);
        } else servo.setPosition(0.38);
    }
    public void resetUnloadMacro() {
        time = null;
    }
}

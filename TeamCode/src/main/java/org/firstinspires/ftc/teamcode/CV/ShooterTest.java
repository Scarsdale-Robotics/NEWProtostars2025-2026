package org.firstinspires.ftc.teamcode.CV;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "IndivShooterTest")
public class ShooterTest extends LinearOpMode {
    public Motor shooter;
    public Motor shooter2;
    @Override
    public void runOpMode() throws InterruptedException {
        this.shooter = new Motor(hardwareMap, "shooter", Motor.GoBILDA.RPM_1620);
        this.shooter2 = new Motor(hardwareMap, "shooter2", Motor.GoBILDA.RPM_1620);
        shooter.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //transfer.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //transfer.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter2.setRunMode(Motor.RunMode.RawPower);
        //transfer.setRunMode(Motor.RunMode.RawPower);

        shooter.setInverted(true);
        shooter2.setInverted(true);

        shooter.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //transfer.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.triangle) shooter.set(0.6);
            else if (gamepad1.circle) shooter.set(-0.6);
            else shooter.set(0);
            if (gamepad1.square) shooter2.set(0.6);
            else if (gamepad1.cross) shooter2.set(-0.6);
            else shooter2.set(0);
        }
    }
}

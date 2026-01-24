package org.firstinspires.ftc.teamcode.CV;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp (name = "Shtest1/23")
public class ShooterTestt extends LinearOpMode {
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

        shooter.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //transfer.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setInverted(true);
        shooter2.setInverted(true);

        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_down) shooter.set(1);
            else shooter.set(0);
            if (gamepad1.dpad_left) shooter2.set(1);
            else shooter2.set(0);
            telemetry.addData("Sh1 vel", shooter.getCorrectedVelocity());
            telemetry.addData("Sh2 vel", shooter2.getCorrectedVelocity());
            telemetry.addData("sh1 inversion", shooter.getInverted());
            telemetry.addData("sh2 inversion", shooter2.getInverted());

            telemetry.update();
        }
    }
}

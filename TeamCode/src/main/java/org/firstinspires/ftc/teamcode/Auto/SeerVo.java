package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Configurable
//range = 0.5 to 0.7
@TeleOp(name = "Servo Test")
public class SeerVo extends LinearOpMode {
    public Servo servo;
    public static double val = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        this.servo = hardwareMap.get(Servo.class, "servo");
        waitForStart();
        while (opModeIsActive()) {
            val = gamepad1.left_stick_y;
            if (val < 0) val = 0;
            servo.setPosition(val);
        }
    }
}

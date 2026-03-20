package org.firstinspires.ftc.teamcode.TeleOp;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Configurable
@TeleOp(name = "ServoTurretTest3/18")
public class TestServoTurret extends LinearOpMode {
    //servo 1 = 0
    //servo 2 = 0.99
    public Servo servo1;
    public Servo servo2;
    public static double pos1 = 0.99;
    public static double phi = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        this.servo1 = hardwareMap.get(Servo.class, "servo_one");
        this.servo2 = hardwareMap.get(Servo.class, "servo_two");
        waitForStart();
        while (opModeIsActive()) {
            servo1.setPosition(0.99 - pos1);
            servo2.setPosition(pos1);
            //setTargetPhi(phi);
            telemetry.addData("Servo 1 Phi", servoOnePhi());
            telemetry.addData("Servo 2 Phi", servoTwoPhi());
            telemetry.addData("Servo 1 Pos", servo1.getPosition());
            telemetry.addData("Servo 2 Pos", servo2.getPosition());
            telemetry.update();
        }
    }
    public void setTargetPhi(double phideg){
        double rad = Math.toRadians(phideg);
        double frac = rad / (2 * Math.PI);
        servo1.setPosition(frac);
        servo2.setPosition(0.99 - frac);
    }
    public double servoOnePhi(){
        return 360 * (servo1.getPosition());
    }
    public double servoTwoPhi(){
        return 360 * (servo1.getPosition());
    }
}

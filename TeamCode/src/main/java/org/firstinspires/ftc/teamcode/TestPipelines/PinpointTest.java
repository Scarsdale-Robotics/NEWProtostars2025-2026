package org.firstinspires.ftc.teamcode.TestPipelines;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp (name = "PPTest")
public class PinpointTest extends LinearOpMode {
    public GoBildaPinpointDriver pp;
    public boolean lastP = false;
    public Servo servo;
    @Override
    public void runOpMode() throws InterruptedException {
        this.pp = hardwareMap.get(GoBildaPinpointDriver.class, "PP");
        this.servo = hardwareMap.get(Servo.class, "Servo");
        pp.resetPosAndIMU();
        pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pp.setPosition(new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES,0));
        pp.setOffsets(0,0, DistanceUnit.INCH);
        waitForStart();
        while (opModeIsActive()) {
            pp.update();
            telemetry.addData("X", pp.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y", pp.getPosY(DistanceUnit.INCH));
            telemetry.addData("H", pp.getHeading(AngleUnit.DEGREES));
            boolean p = gamepad1.cross;
            if (p && !lastP) {
                if (servo.getPosition() == 0) servo.setPosition(0.5);
                else servo.setPosition(0);
            }
            lastP = p;
            telemetry.update();
        }
    }
}

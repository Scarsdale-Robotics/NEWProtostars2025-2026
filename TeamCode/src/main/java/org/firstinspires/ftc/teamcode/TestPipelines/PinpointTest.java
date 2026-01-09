package org.firstinspires.ftc.teamcode.TestPipelines;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp (name = "PPTest")
public class PinpointTest extends LinearOpMode {
    public GoBildaPinpointDriver pp;
    @Override
    public void runOpMode() throws InterruptedException {
        this.pp = hardwareMap.get(GoBildaPinpointDriver.class, "PP");
        pp.resetPosAndIMU();
        while (opModeIsActive()) {
            pp.update();
            telemetry.addData("X", pp.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y", pp.getPosY(DistanceUnit.INCH));
            telemetry.addData("H", pp.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.RobotSystem;
@Autonomous(name = "IntakeTest")
public class IntakeTest extends LinearOpMode {
    public RobotSystem robot;
    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        waitForStart();
        while (opModeIsActive()) {
            robot.inDep.setIntake(0.6);
        }
    }
}

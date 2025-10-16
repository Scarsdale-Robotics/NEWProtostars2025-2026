package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CV.MotifPipeline;
import org.firstinspires.ftc.teamcode.Subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.InDepSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Optional;

public class RobotSystem {
    public final CVSubsystem cv;
    public final HardwareRobot hardwareRobot;
    public final DriveSubsystem drive;
    public final LinearOpMode opMode;
    public final InDepSubsystem inDep;
    public RobotSystem(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.hardwareRobot = new HardwareRobot(hardwareMap);
        this.cv = new CVSubsystem(hardwareRobot.cameraName, opMode, hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        this.drive = new DriveSubsystem(
          hardwareRobot.leftFront,
          hardwareRobot.rightFront,
          hardwareRobot.leftBack,
          hardwareRobot.leftFront
        );
        this.opMode = opMode;
        this.inDep = new InDepSubsystem(opMode, hardwareMap);
    }
    public String decode(AprilTagDetection target) {
        if (target.id == 21) {
            return "GPP";
        }
        else if (target.id == 22) {
            return "PGP";
        }
        else if (target.id == 23) {
            return "PPG";
        }
        return "No Tag Found";
    }
}

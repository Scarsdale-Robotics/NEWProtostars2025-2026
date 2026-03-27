package org.firstinspires.ftc.teamcode.Auto;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class PracticeAuto extends LinearOpMode {
    public Motor leftFront;
    public Motor leftBack;
    public Motor rightFront;
    public Motor rightBack;
    public DriveSubsystem drive;
    public Follower follower;
    public Pose startPose = new Pose(0,0,0);
    @Override
    public void runOpMode() throws InterruptedException {
        this.leftFront = new Motor(hardwareMap, "leftFront");
        this.leftBack = new Motor(hardwareMap, "leftFront");
        this.rightFront = new Motor(hardwareMap, "leftFront");
        this.rightBack = new Motor(hardwareMap, "leftFront");
        this.drive = new DriveSubsystem(leftFront, rightFront, leftBack, rightBack);
        this.follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        waitForStart();
        while (opModeIsActive()) {

        }
    }
}

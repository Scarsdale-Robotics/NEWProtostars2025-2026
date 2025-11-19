package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//TODO: tune pedro constants and also configure ftcdashboard
public class HardwareRobot {

    public final Motor leftFront;
    public final Motor rightFront;
    public final Motor leftBack;
    public final Motor rightBack;
    public final WebcamName cameraName;
    public final GoBildaPinpointDriver pinpoint;
    public final Servo intakeControl;
    public final Motor shooterOne;
    public final Motor shooterTwo;
    public final Motor intakeTwo;
    public final Motor intakeOne;

    public HardwareRobot(HardwareMap hardwareMap) {
        leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
        leftBack = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312);
        rightBack = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312);

        leftFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setRunMode(Motor.RunMode.RawPower);
        rightFront.setRunMode(Motor.RunMode.RawPower);
        leftBack.setRunMode(Motor.RunMode.RawPower);
        rightBack.setRunMode(Motor.RunMode.RawPower);

        leftFront.setInverted(true);
        rightFront.setInverted(true);
        leftBack.setInverted(true);
        rightBack.setInverted(false);

        leftFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        intakeOne = new Motor(hardwareMap, "intakeOne", Motor.GoBILDA.RPM_312);
        intakeTwo = new Motor(hardwareMap, "intakeOne", Motor.GoBILDA.RPM_312);
        shooterOne = new Motor(hardwareMap, "shooterOne", Motor.GoBILDA.RPM_1620);
        shooterTwo = new Motor(hardwareMap, "shooterTwo", Motor.GoBILDA.RPM_1620);

        intakeOne.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeTwo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterOne.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterTwo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeOne.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeTwo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterOne.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterTwo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeOne.setRunMode(Motor.RunMode.RawPower);
        intakeTwo.setRunMode(Motor.RunMode.RawPower);
        shooterOne.setRunMode(Motor.RunMode.RawPower);
        shooterTwo.setRunMode(Motor.RunMode.RawPower);

        intakeOne.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeTwo.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterOne.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterTwo.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeControl = hardwareMap.get(Servo.class, "intakeControl");

        cameraName = hardwareMap.get(WebcamName.class,  "Webcam 1");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }
    public void initOdom() {
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-5.5,-5, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }
}
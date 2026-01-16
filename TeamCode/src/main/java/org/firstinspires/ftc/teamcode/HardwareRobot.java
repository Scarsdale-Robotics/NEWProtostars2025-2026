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
    public final Motor turret;
    public final Servo transferServo;
    public final Motor transferMotor;
    public final Motor shooter;
    public final Motor shooter2;
    public final Servo hoodServo;

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

        leftFront.setInverted(false);
        rightFront.setInverted(true);
        leftBack.setInverted(false);
        rightBack.setInverted(false);

        leftFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        turret = new Motor(hardwareMap, "turret", Motor.GoBILDA.RPM_312);
        shooter2 = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_312);
        shooter = new Motor(hardwareMap, "shooter", Motor.GoBILDA.RPM_1620);
        transferMotor = new Motor(hardwareMap, "transferMotor", Motor.GoBILDA.RPM_312);

        turret.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turret.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transferMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret.setRunMode(Motor.RunMode.RawPower);
        shooter2.setRunMode(Motor.RunMode.RawPower);
        shooter.setRunMode(Motor.RunMode.RawPower);
        transferMotor.setRunMode(Motor.RunMode.RawPower);

        turret.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        transferServo = hardwareMap.get(Servo.class, "transferServo");

        cameraName = hardwareMap.get(WebcamName.class,  "Webcam 1");

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }
    public void initOdom() {
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-5.5,-1.5, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }
}
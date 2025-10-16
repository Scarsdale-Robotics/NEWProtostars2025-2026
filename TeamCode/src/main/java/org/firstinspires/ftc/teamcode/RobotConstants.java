package org.firstinspires.ftc.teamcode;

public class RobotConstants {
    public double CLAW_OPEN;
    public double CLAW_CLOSED;
    public double LOW_POWER;
    public double MIDDLE_POWER;
    public double HIGH_POWER;
    private static final float goBILDA_4_BAR_POD    = 19.89436789f; //ticks-per-mm for the goBILDA 4-Bar Pod
    public final double TICKS_PER_REV = 8192; // REV Through Bore encoder
    public final double WHEEL_DIAMETER_INCHES = 2.0; // goBILDA omni
    public final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;
    public final double TICKS_PER_INCH = TICKS_PER_REV / WHEEL_CIRCUMFERENCE;
}

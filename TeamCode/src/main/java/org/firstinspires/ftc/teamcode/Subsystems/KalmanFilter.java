package org.firstinspires.ftc.teamcode.Subsystems;

public class KalmanFilter {
    public double r;
    public double q;
    public KalmanFilter(double r, double q){
        this.r = r;
        this.q = q;
    }
    public double lastX = 0;
    public double lastP = 0;
    public void update(double ut, double zt) {
        double measurement = lastX + ut;
        double uncertainty = lastP + q;
        double kt = uncertainty / (uncertainty + r);
        measurement = lastX + kt * (zt - measurement);
        uncertainty = (1 - kt) * uncertainty;
        lastP = uncertainty;
        lastX = measurement;
    }
    public double getState() {return lastX;}
}

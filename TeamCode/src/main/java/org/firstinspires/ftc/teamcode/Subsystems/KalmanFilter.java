package org.firstinspires.ftc.teamcode.Subsystems;

public class KalmanFilter {
    public double r;
    public double q;
    public KalmanFilter(double r, double q){
        this.r = r;
        this.q = q;
    }
    public double lastx = 0;
    public double lastp = 0;
    public void update(double ut, double zt) {
        double measurement = lastx + ut;
        double uncertainty = lastp + q;
        double kt = uncertainty / (uncertainty + r);
        measurement = lastx + kt * (zt - measurement);
        uncertainty = (1 - kt) * uncertainty;
        lastp = uncertainty;
        lastx = measurement;
    }
    public double getState() {return lastx;}
}

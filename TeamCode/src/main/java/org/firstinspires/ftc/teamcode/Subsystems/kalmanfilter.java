package org.firstinspires.ftc.teamcode.Subsystems;

public class kalmanfilter {
    public double r;
    public double q;
    public kalmanfilter(double r, double q){
        this.r = r;
        this.q = q;
    }
    public double ut;
    public double kt;
    public double lastx = 0;
    public double lastp = 0;
    public void update(double ut) {
        double measurement = lastx + ut;
        double uncertainty = lastp + q;
        double kt = uncertainty / (uncertainty + r);
        measurement =
        lastp = uncertainty;
        lastx = measurement;
    }
    public double getState(){

    }
}

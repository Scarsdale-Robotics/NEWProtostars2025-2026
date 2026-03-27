package org.firstinspires.ftc.teamcode.Subsystems;
import java.util.List;
public class RollingAverage {
    public List<Double> values;
    public double w1;
    public double w2;
    public double w3;
    public int history;
    public double[] weights;
    public RollingAverage(List<Double> values,int hist, double[] weights) {
        this.values = values;
        this.history = hist;
        this.weights = weights;
    }
    public double getEstimate() {
        double result = 0;
        if (values.isEmpty()) return 0;
        else {
            if (values.size() < history) return values.get(values.size() - 1);
            else {
                int pointer = history - 1;
                int pointer2 = values.size() - 1;
                while (pointer >= 0) {
                    result += weights[pointer] * values.get(pointer2);
                    pointer2--;
                    pointer--;
                }
            }
        }
        return result;
    }
}

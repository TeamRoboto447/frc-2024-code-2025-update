package frc.robot.utils;

import java.util.LinkedList;
import java.util.Queue;

public class RollingAverage {
    private int size;
    private int totalSamples = 0;
    private double sum;
    private Queue<Double> samples;

    public RollingAverage(int size) {
        this.size = size;
        samples = new LinkedList<>();
    }

    public void add(double x) {
        this.samples.add(x);
        this.sum += x;
        if (totalSamples == size) {
            Double y = this.samples.remove();
            this.sum -= y;
        } else
            this.totalSamples += 1;
    }

    public void reset() {
        this.sum = 0;
        this.totalSamples = 0;
        this.samples.clear();
    }

    public double getAvg() {
        return sum / totalSamples;
    }
}

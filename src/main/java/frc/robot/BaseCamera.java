package frc.robot;

import edu.wpi.first.wpilibj.command.Subsystem;

public abstract class BaseCamera extends Subsystem {
    protected abstract double getLeftX();
    protected abstract double getRightX();

    /**
     * Returns the distance from the camera to the centerpoint.
     */
    public abstract double getD();

    /**
     * Returns the points.
     * 
     * l = ([0][0], [0][1])
     * r = ([1][0], [1][1])
     */
    public double[][] getPoints() {
        double[][] output = new double[2][2];

        double l = this.getLeftX();
        double r = this.getRightX();

        double g1 = Constants.TAPE_POINT_DISTANCE / 2;
        double h1 = r - l;
        
        double lambda = Math.sin(Math.acos(h1 / g1)) * g1;

        double dc = this.getD();

        output[0][0] = l;
        output[0][1] = dc - lambda;

        output[1][0] = r;
        output[0][1] = dc + lambda;

        return output;
    }
}
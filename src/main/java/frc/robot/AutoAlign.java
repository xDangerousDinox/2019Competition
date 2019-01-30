package frc.robot;

public final class AutoAlign {

    private double xl;
    private double yl;
    private double xr;
    private double yr;

    private double d;

    private double xc;
    private double yc;

    private double theta_A;
    private double theta_B;
    private double a;
    private double b;

    public AutoAlign(double[][] points, double d) {
        this(points[0], points[1], d);
    }

    public AutoAlign(double[] l, double[] r, double d) {
        this(l[0], l[1], r[0], r[1], d);
    }

    public AutoAlign(double xl, double yl, double xr, double yr, double d) {
        this.xl = xl;
        this.yl = yl;
        this.xr = xr;
        this.yr = yr;
        this.d = d;

        double[] center = AutoAlign.getCenterPoint(this.xl, this.yl, this.xr, this.yr);
        this.xc = center[0];
        this.yc = center[1];

        this.theta_A = Math.atan((this.yc - this.yl) / (this.xc - this.xl));
        this.theta_B = 90 - this.theta_A;
        this.a = Math.cos(this.theta_B) * this.d;
        this.b = Math.sin(this.theta_B) * this.d;
    }

    public double getThetaB() {
        return this.theta_B;
    }

    public double getA() {
        return this.a;
    }

    public double getB() {
        return this.b;
    }

    public static double getDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2));
    }

    public static double[] getCenterPoint(double x1, double y1, double x2, double y2) {
        double[] output = new double[2];
        output[0] = (x1 + x2) / 2;
        output[1] = (y1 + y2) / 2;
        return output;
    }

    public static double getError(double xl, double yl, double xr, double yr) {
        double[] centerpoint = AutoAlign.getCenterPoint(xl, yl, xr, yr);
        return AutoAlign.getDistance(xl, yl, centerpoint[0], centerpoint[1])
         - AutoAlign.getDistance(xr, yr, centerpoint[0], centerpoint[1]);
    }

}
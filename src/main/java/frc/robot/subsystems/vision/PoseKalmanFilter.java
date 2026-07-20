package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Layer 2 of pose estimation: a minimal diagonal Kalman filter over robot pose (x, y, theta).
 *
 * <p>The prediction step is driven by the drive's odometry delta; the correction step pulls the
 * estimate toward the inverse-variance-fused limelight pose ({@link VisionFusion}). Process noise
 * controls responsiveness — larger process noise grows the estimate covariance faster, raising the
 * Kalman gain so the estimate chases the vision measurement more quickly.
 */
public class PoseKalmanFilter {

    private double x;
    private double y;
    private double theta;

    // Estimate variances (diagonal covariance).
    private double px;
    private double py;
    private double pTheta;

    // Process noise (variance) added each prediction.
    private double qx;
    private double qy;
    private double qTheta;

    private boolean initialized = false;

    public PoseKalmanFilter(double processStdTranslation_m, double processStdRotation_rad) {
        setProcessNoise(processStdTranslation_m, processStdRotation_rad);
    }

    /** Set the per-loop process noise. Larger values make the estimate converge to vision faster. */
    public void setProcessNoise(double stdTranslation_m, double stdRotation_rad) {
        this.qx = stdTranslation_m * stdTranslation_m;
        this.qy = this.qx;
        this.qTheta = stdRotation_rad * stdRotation_rad;
    }

    public boolean isInitialized() {
        return initialized;
    }

    public void reset(Pose2d pose) {
        x = pose.getX();
        y = pose.getY();
        theta = pose.getRotation().getRadians();
        px = 0.0;
        py = 0.0;
        pTheta = 0.0;
        initialized = true;
    }

    public void predict(double dx, double dy, double dTheta) {
        if (!initialized) {
            return;
        }
        x += dx;
        y += dy;
        theta += dTheta;
        px += qx;
        py += qy;
        pTheta += qTheta;
    }

    public void update(Pose2d measurement, double rx, double ry, double rTheta) {
        if (!initialized) {
            reset(measurement);
            px = rx;
            py = ry;
            pTheta = rTheta;
            return;
        }

        double kx = px / (px + rx);
        double ky = py / (py + ry);
        double kTheta = pTheta / (pTheta + rTheta);

        x += kx * (measurement.getX() - x);
        y += ky * (measurement.getY() - y);
        theta += kTheta * MathUtil.angleModulus(measurement.getRotation().getRadians() - theta);

        px *= (1.0 - kx);
        py *= (1.0 - ky);
        pTheta *= (1.0 - kTheta);
    }

    public Pose2d getEstimate() {
        return new Pose2d(x, y, new Rotation2d(theta));
    }

    public double getStdDevX() {
        return Math.sqrt(px);
    }

    public double getStdDevY() {
        return Math.sqrt(py);
    }

    public double getStdDevTheta() {
        return Math.sqrt(pTheta);
    }
}

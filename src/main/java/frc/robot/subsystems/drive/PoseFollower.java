package frc.robot.subsystems.drive;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.Util;

/**
 * Follows a target pose with smooth motion.
 * 
 * Uses constant velocity (maxSpeed) for translation, with P-control only for direction.
 * This prevents slowdown at intermediate waypoints - the robot maintains cruise speed
 * and only slows via the acceleration limiter when the target changes direction.
 * 
 * Rotation still uses P-control for smooth heading tracking.
 */
public class PoseFollower {

    private double translate_kP = 3.0;  // Used for direction blending, not speed
    private double rotate_kP = 4.0;

    private double maxSpeed = Drive.MAX_LINEAR_VEL_mps;
    private Pose2d targetPose = new Pose2d();
    
    // Smoothing - limit how fast velocity can change
    private static final double DEFAULT_MAX_ACCEL = Drive.MAX_AUTO_FORWARD_ACC_mps2;  // m/s^2 - max linear acceleration
    private static final double MAX_ANGULAR_ACCEL = Drive.MAX_ANGULAR_VEL_radps;  // rad/s^2 - max angular acceleration
    private static final double DT = 0.02;  // 20ms loop time
    
    private double maxAccel = 200;  // configurable per-instance
    
    // Distance threshold for slowing down (only near final target)
    private static final double STOP_DISTANCE = 0.10;     // Stop commanding at 10cm (settled)
    
    // Hang precision mode: uses P-control with a tighter dead zone instead of
    // the default constant-speed + 10cm cutoff. Only affects hang auto-align.
    private boolean hangPrecisionMode = false;
    private static final double HANG_STOP_DISTANCE = 0.03;  // 3cm dead zone for hang
    
    private double lastVx = 0;
    private double lastVy = 0;
    private double lastOmega = 0;

    public PoseFollower(Pose2d targetPose, double maxSpeed, double translate_kp, double rotate_kP) {
        setParams(targetPose, maxSpeed, translate_kp, rotate_kP);
    }

    public PoseFollower(Pose2d targetPose, double maxSpeed) {
        this.targetPose = targetPose;
        this.maxSpeed = maxSpeed;
    }
    
    public void setParams(Pose2d targetPose, double maxSpeed) {
        setParams(targetPose, maxSpeed, translate_kP, rotate_kP);
    }

    public void setParams(Pose2d targetPose, double maxSpeed, double translate_kp, double rotate_kP) {
        this.targetPose = targetPose;
        this.maxSpeed = maxSpeed;
        this.translate_kP = translate_kp;
        this.rotate_kP = rotate_kP;
    }
    
    /**
     * Reset the velocity smoothing (call when starting a new path)
     */
    public void reset() {
        lastVx = 0;
        lastVy = 0;
        lastOmega = 0;
        hangPrecisionMode = false;
    }

    /**
     * Set the max linear acceleration (m/s^2).
     * Use this to override the default for auto-specific tuning.
     */
    public void setMaxAccel(double maxAccelMps2) {
        this.maxAccel = maxAccelMps2;
    }

    /**
     * Reset max acceleration to the default value.
     */
    public void resetMaxAccel() {
        this.maxAccel = DEFAULT_MAX_ACCEL;
    }

    /**
     * Enable hang precision mode: uses P-control (translate_kP * distance) with
     * a 3cm dead zone instead of the default constant-speed + 10cm cutoff.
     * This allows the robot to smoothly approach hang waypoints with high precision.
     */
    public void enableHangPrecisionMode() {
        this.hangPrecisionMode = true;
    }

    /**
     * Disable hang precision mode, restoring default translation behavior.
     */
    public void disableHangPrecisionMode() {
        this.hangPrecisionMode = false;
    }

    public ChassisSpeeds process(Pose2d currentPose) {
        Transform2d poseDiff = targetPose.minus(currentPose);

        double distance = Math.hypot(poseDiff.getX(), poseDiff.getY());

        // Calculate desired angular velocity (still P-control for smooth rotation)
        Rotation2d rotationDiff = poseDiff.getRotation();
        double radDiff = rotationDiff.getRadians();
        double desiredOmega = 
                rotate_kP * radDiff * Drive.MAX_ANGULAR_VEL_radps;
        

        // Calculate desired linear velocity
        double desiredVx = 0;
        double desiredVy = 0;
        
        double effectiveStopDistance = hangPrecisionMode ? HANG_STOP_DISTANCE : STOP_DISTANCE;
        
        if (distance > effectiveStopDistance) {
            // Direction toward target (normalized)
            double dirX = poseDiff.getX() / distance;
            double dirY = poseDiff.getY() / distance;
            
            double speed;
            if (hangPrecisionMode) {
                // P-control: smooth deceleration all the way to 3cm dead zone
                speed = Util.limit(translate_kP * distance, maxSpeed);
            } else {
                // Default P-control deceleration mechanism. Since the target is effectively 0.75m ahead 
                // of the robot for mid-path waypoints, this runs at mostly full power throughout the
                // journey and perfectly dampens via `translate_kP` ONLY at the very end.
                speed = Util.limit(translate_kP * distance, maxSpeed);
            }
            
            desiredVx = dirX * speed;
            desiredVy = dirY * speed;
        }
        // else: within effectiveStopDistance, desiredVx/Vy stay at 0 (stop)
        
        // Apply acceleration limiting for smooth motion
        double maxLinearDelta = maxAccel * DT;
        // Don't cap angular accel harshly per tick, let Drive's chassis acceleration limiters handle it smoothly
        double maxAngularDelta = MAX_ANGULAR_ACCEL * DT;
        
        double vx = rampValue(lastVx, desiredVx, maxLinearDelta);
        double vy = rampValue(lastVy, desiredVy, maxLinearDelta);
        double omega = desiredOmega;
        
        // Store for next iteration
        lastVx = vx;
        lastVy = vy;
        lastOmega = omega;

        return new ChassisSpeeds(vx, vy, omega);
    }
    
    /**
     * Ramp a value toward a target, limited by maxDelta per cycle
     */
    private double rampValue(double current, double target, double maxDelta) {
        double diff = target - current;
        if (Math.abs(diff) <= maxDelta) {
            return target;
        }
        return current + Math.signum(diff) * maxDelta;
    }
}

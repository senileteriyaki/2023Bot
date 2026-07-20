package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import frc.robot.Constants;
import frc.robot.devices.limelight.Limelight;
import frc.robot.devices.limelight.LimelightConfig;
import frc.robot.devices.limelight.LimelightIO.LimelightIOInputs.PoseObservation;
import frc.robot.devices.limelight.LimelightIO.LimelightIOInputs.PoseType;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionFusion.Measurement;

public class Vision extends SubsystemBase<Vision.Command> {

    public enum Command {
        DISABLED,
        ENABLED
    }

    private static Vision instance;

    private final List<Limelight> limelights = new ArrayList<>();
    private final AprilTagFieldLayout tagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private final Drive drive;
    private final PoseKalmanFilter filter = new PoseKalmanFilter(
            VisionConstants.PROCESS_STD_TRANSLATION_m, VisionConstants.PROCESS_STD_ROTATION_rad);

    private Pose2d fusedPose = null;
    private Pose2d filteredPose = null;
    private int acceptedThisCycle = 0;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
            System.out.println("Vision initialized.");
        }
        return instance;
    }

    private Vision() {
        super("Vision");
        for (VisionConstants.CameraConfig camera : VisionConstants.CAMERAS) {
            limelights.add(new Limelight(
                    "Vision/" + camera.name(),
                    new LimelightConfig(camera.name())
                            .withRobotToCamera(camera.robotToCamera())
                            .withMaxSightRange(VisionConstants.SIM_MAX_SIGHT_RANGE_m)));
        }
        drive = Drive.getInstance();
        setCommand(Command.DISABLED);
    }

    @Override
    protected void inputPeriodic() {
        for (Limelight limelight : limelights) {
            limelight.setRobotOrientation(drive.getRotation());
            limelight.updateSimPose(drive.getPose());
            limelight.readInputs();
        }
    }

    @Override
    protected void handle() {
        fusedPose = null;
        filteredPose = null;
        acceptedThisCycle = 0;

        // Layer 2 prediction always runs so the filter tracks the robot between vision frames.
        predictFromOdometry();

        // Layer 1: collect every accepted observation and inverse-variance fuse them.
        Measurement fused = VisionFusion.fuse(collectMeasurements());
        if (fused == null || getCommand() == Command.DISABLED) {
            filteredPose = filter.isInitialized() ? filter.getEstimate() : null;
            return;
        }
        fusedPose = fused.pose();

        // Layer 2 correction: pull the filter toward the fused vision pose.
        filter.update(fused.pose(), fused.varX(), fused.varY(), fused.varTheta());
        filteredPose = filter.getEstimate();

        // Layer 3: hand the filtered pose to the drive pose estimator.
        drive.addVisionMeasurement(filteredPose, fused.timestamp(), driveStdDevs());
    }

    @Override
    protected void outputPeriodic() {
        Logger.recordOutput("Vision/AcceptedObservations", acceptedThisCycle);
        Logger.recordOutput("Vision/HasFusedPose", fusedPose != null);
        if (fusedPose != null) {
            Logger.recordOutput("Vision/FusedPose", fusedPose);
        }
        if (filteredPose != null) {
            Logger.recordOutput("Vision/FilteredPose", filteredPose);
        }
        logSightlines();
    }

    // Log lines from each camera to the tags it sees, so you can watch what vision is looking at in
    // AdvantageScope (add the "Sightlines" fields to a 3D field view).
    private void logSightlines() {
        Pose3d robot = new Pose3d(drive.getPose());

        List<Pose3d> allTagPoses = new ArrayList<>();   // every visible tag, all cameras.
        List<Pose3d> allSightlines = new ArrayList<>(); // camera->tag line segments, all cameras.
        for (Limelight limelight : limelights) {
            Pose3d camera = robot.transformBy(limelight.getRobotToCamera()); // where this lens is.
            List<Pose3d> tagPoses = new ArrayList<>();
            List<Pose3d> sightlines = new ArrayList<>();
            for (int tagId : limelight.getVisibleTagIds()) {
                tagLayout.getTagPose(tagId).ifPresent(tagPose -> {
                    tagPoses.add(tagPose);
                    // A line = two points in a row: camera, then the tag.
                    sightlines.add(camera);
                    sightlines.add(tagPose);
                });
            }
            Logger.recordOutput(limelight.getName() + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
            Logger.recordOutput(limelight.getName() + "/Sightlines", sightlines.toArray(new Pose3d[0]));
            allTagPoses.addAll(tagPoses);
            allSightlines.addAll(sightlines);
        }
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/Sightlines", allSightlines.toArray(new Pose3d[0]));
    }

    // ---- Layer 1: build per-limelight measurements ----

    private List<Measurement> collectMeasurements() {
        List<Measurement> measurements = new ArrayList<>();
        for (Limelight limelight : limelights) {
            for (PoseObservation observation : limelight.getPoseObservations()) {
                if (shouldReject(observation)) {
                    continue;
                }
                measurements.add(toMeasurement(observation));
                acceptedThisCycle++;
            }
        }
        return measurements;
    }

    private boolean shouldReject(PoseObservation observation) {
        if (observation.tagCount() == 0) {
            return true;
        }
        if (observation.tagCount() == 1 && observation.ambiguity() > VisionConstants.MAX_AMBIGUITY) {
            return true;
        }
        if (Math.abs(observation.pose().getZ()) > VisionConstants.MAX_Z_ERROR_m) {
            return true;
        }
        Pose2d pose = observation.pose().toPose2d();
        return pose.getX() < 0.0 || pose.getY() < 0.0;
    }

    private Measurement toMeasurement(PoseObservation observation) {
        double factor = Math.pow(observation.avgTagDistance(), 2.0)
                / Math.max(observation.tagCount(), 1);

        double linearStd = VisionConstants.LINEAR_STD_BASELINE_m * factor;
        double rotationStd = observation.type() == PoseType.MEGATAG_2
                ? VisionConstants.MEGATAG2_ROTATION_STD_rad
                : VisionConstants.ANGULAR_STD_BASELINE_rad * factor;

        return new Measurement(
                observation.pose().toPose2d(),
                linearStd * linearStd,
                linearStd * linearStd,
                rotationStd * rotationStd,
                observation.timestamp());
    }

    // ---- Layer 2: prediction + std devs handed to the drive ----

    private void predictFromOdometry() {
        if (!filter.isInitialized()) {
            return;
        }
        ChassisSpeeds robotRelative = drive.getChassisSpeeds();
        Translation2d fieldVelocity = new Translation2d(
                robotRelative.vxMetersPerSecond, robotRelative.vyMetersPerSecond)
                .rotateBy(drive.getRotation());
        double dt = Constants.globalDelta_s;
        filter.predict(
                fieldVelocity.getX() * dt,
                fieldVelocity.getY() * dt,
                robotRelative.omegaRadiansPerSecond * dt);
    }

    private Matrix<N3, N1> driveStdDevs() {
        double stdX = Math.max(filter.getStdDevX(), VisionConstants.MIN_DRIVE_STD_TRANSLATION_m);
        double stdY = Math.max(filter.getStdDevY(), VisionConstants.MIN_DRIVE_STD_TRANSLATION_m);
        double stdTheta = Math.max(filter.getStdDevTheta(), VisionConstants.MIN_DRIVE_STD_ROTATION_rad);
        return VecBuilder.fill(stdX, stdY, stdTheta);
    }

    // ---- Public API ----

    public void enable() {
        setCommand(Command.ENABLED);
    }

    public void disable() {
        setCommand(Command.DISABLED);
    }

    /**
     * Tune how quickly the filtered pose chases the vision target. Larger values move the estimate
     * toward the fused limelight pose faster; smaller values lean on odometry.
     */
    public void setResponsiveness(double processStdTranslation_m, double processStdRotation_rad) {
        filter.setProcessNoise(processStdTranslation_m, processStdRotation_rad);
    }

    public boolean hasFusedPose() {
        return fusedPose != null;
    }

    /** The inverse-variance-fused limelight pose from this cycle, or null. */
    public Pose2d getFusedPose() {
        return fusedPose;
    }

    /** The Kalman-filtered pose from this cycle, or null before the filter initializes. */
    public Pose2d getFilteredPose() {
        return filteredPose;
    }
}

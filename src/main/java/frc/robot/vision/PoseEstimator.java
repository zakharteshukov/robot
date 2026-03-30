package frc.robot.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class PoseEstimator extends SubsystemBase {

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Supplier<Rotation2d> gyroSupplier;

  private final Supplier<SwerveModulePosition[]> modulePositionSupplier;

  private static final String LIMELIGHT_NAME = Constants.limelight_name;

  private final Field2d field = new Field2d();

  public static boolean visionEnable;

  public PoseEstimator(

      SwerveDriveKinematics kinematics,

      Supplier<Rotation2d> gyroSupplier,

      Supplier<SwerveModulePosition[]> modulePositionSupplier,

      Pose2d initialPose,

      boolean visionEnabled

  )

  {

    this.gyroSupplier = gyroSupplier;

    this.modulePositionSupplier = modulePositionSupplier;

    PoseEstimator.visionEnable = visionEnabled;

    poseEstimator = new SwerveDrivePoseEstimator(

        kinematics,

        gyroSupplier.get(),

        modulePositionSupplier.get(),

        initialPose,

        // State StdDevs (encoder + gyro güveni)

        VecBuilder.fill(0.05, 0.05, Math.toRadians(2)),

        // Vision StdDevs (ilk değer, dinamik ayarlanacak)

        VecBuilder.fill(0.7, 0.7, Math.toRadians(15))

    );

    LimelightHelpers.SetIMUMode(LIMELIGHT_NAME, 1);
    SmartDashboard.putData("Field", field);
    field.setRobotPose(poseEstimator.getEstimatedPosition());

  }

  public void updateLimelightOrientation() {

    double rollRate = 0.0;

    double pitchRate = 0.0;

    double yawRate = -Constants.gyro.getRate(); // (deg/sec)

    double roll = 0.0;

    double pitch = 0.0;

    // double yaw = Math.IEEEremainder(-Constants.gyro.getYaw() , 360);

    double yaw = poseEstimator.getEstimatedPosition().getRotation().getDegrees();

    LimelightHelpers.SetRobotOrientation(

        Constants.limelight_name,

        yaw,

        yawRate,

        pitch,

        pitchRate,

        roll,

        rollRate

    );

  }

  private void addLimelightVision() {

    // if (!visionEnable) return;

    // var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);

    var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);

    if (estimate == null || estimate.pose == null)
      return;

    Pose2d visionPose = estimate.pose;

    double timestamp = estimate.timestampSeconds;

    int tagCount = estimate.tagCount;

    double avgTagDist = estimate.avgTagDist;

    if (Double.isNaN(visionPose.getX()) || Double.isNaN(visionPose.getY()))
      return;

    if (Double.isInfinite(visionPose.getX()) || Double.isInfinite(visionPose.getY()))
      return;

    if (avgTagDist > 6 || tagCount == 0)
      return;

    double yawRate = Math.abs(-Constants.gyro.getRate());

    if (yawRate > 30.0)
      return;

    // Dinamik güven

    double xyStdDev = 0.125 * Math.pow(avgTagDist, 2.0) / tagCount;

    poseEstimator.setVisionMeasurementStdDevs(

        VecBuilder.fill(xyStdDev, xyStdDev, 9999999.0)

    );

    poseEstimator.addVisionMeasurement(

        visionPose,

        timestamp

    );

  }

  public Pose2d getPose() {

    return poseEstimator.getEstimatedPosition();

  }

  public void resetPose(Pose2d pose) {

    poseEstimator.resetPosition(

        gyroSupplier.get(),

        modulePositionSupplier.get(),

        pose

    );

  }

  @Override

  public void periodic() {

    updateLimelightOrientation();

    poseEstimator.update(

        gyroSupplier.get(),

        modulePositionSupplier.get()

    );

    addLimelightVision();

    field.setRobotPose(getPose());

  }
}
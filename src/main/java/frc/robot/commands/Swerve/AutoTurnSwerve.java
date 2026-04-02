package frc.robot.commands.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveSubsystemConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/**
 * Turns in place using the swerve drive. Completion is based on estimated pose rotation
 * (gyro + wheel odometry from {@link frc.robot.vision.PoseEstimator}), not wheel arc
 * length like a tank drive.
 *
 * <p>Sign convention matches tank {@code AutoTurn}: positive {@code targetAngleDegrees}
 * = turn right (clockwise when viewed from above).
 *
 * @see SwerveDriveSubsystemConstants#AutoMaxAngularVelocityRadPerSec
 * @see SwerveDriveSubsystemConstants#AutoTurnAngleToleranceDeg
 * @see SwerveDriveSubsystemConstants#AutoTurnAngleScalar
 */
public class AutoTurnSwerve extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final double targetAngleDegrees;
  private final double turnSpeed;
  private Rotation2d startRotation;

  /**
   * @param swerveSubsystem      Swerve subsystem (pose rotation is used).
   * @param targetAngleDegrees   Positive = turn right, negative = turn left.
   * @param turnSpeed            Turn rate magnitude in [0, 1] relative to
   *                             {@link SwerveDriveSubsystemConstants#AutoMaxAngularVelocityRadPerSec}.
   */
  public AutoTurnSwerve(
      SwerveSubsystem swerveSubsystem, double targetAngleDegrees, double turnSpeed) {
    addRequirements(swerveSubsystem);
    this.swerveSubsystem = swerveSubsystem;
    this.targetAngleDegrees = targetAngleDegrees;
    this.turnSpeed = Math.abs(turnSpeed);
  }

  @Override
  public void initialize() {
    startRotation = swerveSubsystem.getPose().getRotation();
  }

  @Override
  public void execute() {
    double sign = Math.signum(targetAngleDegrees);
    double omega = -sign * turnSpeed * SwerveDriveSubsystemConstants.AutoMaxAngularVelocityRadPerSec;
    ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, omega);
    SwerveModuleState[] states =
        Constants.SwerveDriveSubsystemConstants.kinematics.toSwerveModuleStates(speeds);
    swerveSubsystem.setState(states);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    if (Math.abs(targetAngleDegrees) < 1e-6) {
      return true;
    }
    double targetRad =
        Math.toRadians(Math.abs(targetAngleDegrees))
            * SwerveDriveSubsystemConstants.AutoTurnAngleScalar;
    Rotation2d current = swerveSubsystem.getPose().getRotation();
    double deltaRad =
        MathUtil.inputModulus(
            current.getRadians() - startRotation.getRadians(), -Math.PI, Math.PI);
    // Positive when rotating in the commanded direction (right = negative delta in CCW frame).
    double progressedRad = -Math.signum(targetAngleDegrees) * deltaRad;
    return progressedRad
        >= targetRad - Math.toRadians(SwerveDriveSubsystemConstants.AutoTurnAngleToleranceDeg);
  }
}

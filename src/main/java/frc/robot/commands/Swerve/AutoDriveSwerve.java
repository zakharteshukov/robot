package frc.robot.commands.Swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveSubsystemConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/**
 * Drives the swerve chassis at fixed robot-relative speeds for autonomous.
 *
 * <p>Speeds follow the same sign convention as tank arcade helpers in this project:
 * forward ({@code xSpeed}) positive = drive forward; rotation ({@code zRotation})
 * positive = turn right. WPILib uses counterclockwise-positive omega, so rotation is
 * mapped internally.
 *
 * <p>Normalized inputs are scaled by {@link Constants.ModuleConstants#LinearMaxSpeed} and
 * {@link SwerveDriveSubsystemConstants#AutoMaxAngularVelocityRadPerSec}. This command
 * never finishes on its own; pair with {@code .withTimeout(seconds)} or sequence it.
 *
 * <p>Example: drive straight for 2 seconds:
 *
 * <pre>
 * new AutoDriveSwerve(swerve, 0.5, 0.0).withTimeout(2)
 * </pre>
 */
public class AutoDriveSwerve extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final double xSpeed;
  private final double zRotation;

  /**
   * @param swerve      Swerve drive subsystem
   * @param xSpeed      Forward fraction roughly in [-1, 1]; positive = forward
   * @param zRotation   Turn fraction roughly in [-1, 1]; positive = turn right
   */
  public AutoDriveSwerve(SwerveSubsystem swerve, double xSpeed, double zRotation) {
    addRequirements(swerve);
    this.swerveSubsystem = swerve;
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double vx = xSpeed * Constants.ModuleConstants.LinearMaxSpeed;
    double omega = -zRotation * SwerveDriveSubsystemConstants.AutoMaxAngularVelocityRadPerSec;
    ChassisSpeeds speeds = new ChassisSpeeds(vx, 0.0, omega);
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
    return false;
  }
}

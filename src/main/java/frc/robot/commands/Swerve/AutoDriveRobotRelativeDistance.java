package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDriveSubsystemConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/**
 * Drives in a straight line along robot X at {@link Constants.ModuleConstants#LinearMaxSpeed}
 * times the given fraction until odometry shows the chassis has displaced {@code distanceMeters}
 * along the <em>starting</em> heading (forward if {@code xSpeedFraction > 0}, backward if {@code
 * xSpeedFraction < 0}).
 *
 * <p>Uses field pose and projects displacement onto the start forward vector so distance is
 * geometrically one meter along the floor, not a timed guess.
 */
public class AutoDriveRobotRelativeDistance extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final double xSpeedFraction;
  private final double distanceMeters;
  private final double timeoutSeconds;

  private final Timer timeoutTimer = new Timer();
  private Pose2d startPose;

  /**
   * @param xSpeedFraction forward/back fraction in [-1, 1] (same sign convention as {@link
   *     AutoDriveSwerve})
   * @param distanceMeters positive distance along that axis
   * @param timeoutSeconds safety stop if encoders never satisfy distance
   */
  public AutoDriveRobotRelativeDistance(
      SwerveSubsystem swerve,
      double xSpeedFraction,
      double distanceMeters,
      double timeoutSeconds) {
    addRequirements(swerve);
    this.swerveSubsystem = swerve;
    this.xSpeedFraction = xSpeedFraction;
    this.distanceMeters = Math.abs(distanceMeters);
    this.timeoutSeconds = timeoutSeconds;
  }

  @Override
  public void initialize() {
    startPose = swerveSubsystem.getPose();
    timeoutTimer.restart();
  }

  @Override
  public void execute() {
    double vx = xSpeedFraction * Constants.ModuleConstants.LinearMaxSpeed;
    ChassisSpeeds speeds = new ChassisSpeeds(vx, 0.0, 0.0);
    SwerveModuleState[] states =
        SwerveDriveSubsystemConstants.kinematics.toSwerveModuleStates(speeds);
    swerveSubsystem.setState(states);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    if (timeoutTimer.hasElapsed(timeoutSeconds)) {
      return true;
    }
    Translation2d delta =
        swerveSubsystem.getPose().getTranslation().minus(startPose.getTranslation());
    Translation2d forward = new Translation2d(1.0, 0.0).rotateBy(startPose.getRotation());
    double alongStartForward = delta.getX() * forward.getX() + delta.getY() * forward.getY();
    if (xSpeedFraction < 0) {
      return alongStartForward <= -distanceMeters + AutoConstants.AutonDistanceEpsilonMeters;
    }
    return alongStartForward >= distanceMeters - AutoConstants.AutonDistanceEpsilonMeters;
  }
}

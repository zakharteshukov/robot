package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.FieldZones;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/**
 * Holds the robot still and runs flywheel, hood, feeder, and hopper for a fixed time.
 * Flywheel RPM is chosen from odometry distance to the hub using the same lookup as
 * {@link CalculateFromDistanceCmd}; hood angle uses {@link AutoConstants#BasicShootHoodDegrees}.
 */
public class AutoBasicShootCmd extends Command {

  /**
   * Sets flywheel RPM from hub distance and hood angle for basic auto. Does not run feeder,
   * hopper, or drivetrain; use during auton motion to spin up before the timed shoot.
   */
  public static void applyHubTrackedAim(
      SwerveSubsystem swerveSubsystem,
      FlyWheelSubsystem flyWheelSubsystem,
      HoodSubsystem hoodSubsystem) {
    Translation2d hub = FieldZones.HubZones.getHub().toTranslation2d();
    double distance = hub.getDistance(swerveSubsystem.getPose().getTranslation());
    AngularVelocity flywheelSpeed = flyWheelSubsystem.calculateRpm(distance);
    flyWheelSubsystem.setFlywheelSpeed(flywheelSpeed);
    hoodSubsystem.setHoodAngle(Degrees.of(AutoConstants.BasicShootHoodDegrees));
  }

  private final FlyWheelSubsystem flyWheelSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final HopperSubsystem hopperSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final double durationSeconds;
  private final Timer timer = new Timer();

  public AutoBasicShootCmd(
      FlyWheelSubsystem flyWheelSubsystem,
      HoodSubsystem hoodSubsystem,
      FeederSubsystem feederSubsystem,
      HopperSubsystem hopperSubsystem,
      SwerveSubsystem swerveSubsystem) {
    this(
        flyWheelSubsystem,
        hoodSubsystem,
        feederSubsystem,
        hopperSubsystem,
        swerveSubsystem,
        AutoConstants.BasicShootDurationSeconds);
  }

  public AutoBasicShootCmd(
      FlyWheelSubsystem flyWheelSubsystem,
      HoodSubsystem hoodSubsystem,
      FeederSubsystem feederSubsystem,
      HopperSubsystem hopperSubsystem,
      SwerveSubsystem swerveSubsystem,
      double durationSeconds) {
    this.flyWheelSubsystem = flyWheelSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.hopperSubsystem = hopperSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.durationSeconds = durationSeconds;
    addRequirements(
        flyWheelSubsystem, hoodSubsystem, feederSubsystem, hopperSubsystem, swerveSubsystem);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    swerveSubsystem.stopModules();

    applyHubTrackedAim(swerveSubsystem, flyWheelSubsystem, hoodSubsystem);

    feederSubsystem.FeedMotorSet(AutoConstants.BasicShootFeeder);
    hopperSubsystem.HopperMotorSet(AutoConstants.BasicShootHopper);
  }

  @Override
  public void end(boolean interrupted) {
    flyWheelSubsystem.setFlywheelSpeed(RPM.of(0));
    feederSubsystem.FeedMotorSet(0);
    hopperSubsystem.HopperMotorSet(0);
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(durationSeconds);
  }
}

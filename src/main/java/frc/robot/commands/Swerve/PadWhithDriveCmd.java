package frc.robot.commands.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class PadWhithDriveCmd extends Command {
  private final DoubleSupplier xSupplier, ySupplier, azimuthSupplier;
  private final BooleanSupplier fieldOrientedSupplier;
  private final SwerveSubsystem swerveSubsystem;
  private final SlewRateLimiter xLimiter, yLimiter, azimuthLimiter;

  public PadWhithDriveCmd(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier azimuthSupplier,
      BooleanSupplier fieldOrientedSupplier, SwerveSubsystem swerveSubsystem) {
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.azimuthSupplier = azimuthSupplier;
    this.fieldOrientedSupplier = fieldOrientedSupplier;
    this.swerveSubsystem = swerveSubsystem;
    xLimiter = new SlewRateLimiter(Constants.SwerveDriveSubsystemConstants.SlewRateXLimit);
    yLimiter = new SlewRateLimiter(Constants.SwerveDriveSubsystemConstants.SlewRateYLimit);
    azimuthLimiter = new SlewRateLimiter(Constants.SwerveDriveSubsystemConstants.SlewRateAzimuthLimit);
    addRequirements(swerveSubsystem);

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double xSpeed = xSupplier.getAsDouble();
    double ySpeed = ySupplier.getAsDouble();
    double azimuth = azimuthSupplier.getAsDouble();

    var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;

    // FRC WPILib'de 0 konumu daima Mavi İttifak tabanlıdır.
    // Eğer kırmızı ittifaktaysak eski/normal Xbox değerleri bizi kusursuz sürüyor
    // (-X uzağa gider).
    // Ancak Mavi ittifaktaysak, ileri ittiğimizde +X'e (uzağa) gitmesi için
    // joystiği eksi ile çarpıyoruz.
    if (!isRed) {
      xSpeed = -xSpeed;
      ySpeed = -ySpeed;
    }

    xSpeed = Math.abs(xSpeed) > 0.05 ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > 0.05 ? ySpeed : 0;
    azimuth = Math.abs(azimuth) > 0.05 ? azimuth : 0;

    xSpeed = xLimiter.calculate(xSpeed) * Constants.ModuleConstants.LinearMaxSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * Constants.ModuleConstants.LinearMaxSpeed;
    azimuth = azimuthLimiter.calculate(azimuth) * Constants.ModuleConstants.AngularMaxSpeed;

    ChassisSpeeds speed;

    if (fieldOrientedSupplier.getAsBoolean()) {
      speed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, azimuth, swerveSubsystem.getRotation2d());
    } else {
      speed = new ChassisSpeeds(xSpeed, ySpeed, azimuth);
    }

    SwerveModuleState[] states = Constants.SwerveDriveSubsystemConstants.kinematics.toSwerveModuleStates(speed);

    swerveSubsystem.setState(states);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

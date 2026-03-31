package frc.robot.commands.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    double azimuth = -azimuthSupplier.getAsDouble();

    var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;

    // FRC Sahasında Kırmızı Duvar (Sürücü) +X (16.5m) tarafındadır. Mavi Duvar
    // (Origin) 0'dadır.
    // İleri (Kolu itmek) daima Sürücünün baktığı yön olmalıdır (Kırmızıdan Maviye,
    // Maviden Kırmızıya).
    // Kırmızı Sürücü ileri ittiğinde: X ekseninde 16.5'ten 0'a, yani Negatif (-X)
    // değerine inmelidir. (Xbox kolu zaten -1 verir, bu yüzden birebir uyar!)
    // Mavi Sürücü İleri ittiğinde: X ekseninde 0'dan 16.5'e, yani Pozitif (+X)
    // değerine çıkmalıdır. (Xbox kolu -1 çıkaracağından eksi ile çarpılıp (+)
    // yapılmalıdır!)
    if (!isRed) {
      xSpeed = -xSpeed;
      ySpeed = -ySpeed;
    }

    SmartDashboard.putBoolean("IsBlueAlliance", !isRed);

    // Daha profesyonel ve pürüzsüz bir sürücü deneyimi için MathUtil.applyDeadband
    // kullanıyoruz.
    // Düz Math.abs tetiği 0.11'e geldiğinde anında %11 güç verip zıplatır.
    // applyDeadband ise 0.11'de 0'dan yeni başlar ve %1 güç verir. Çok daha
    // yumuşaktır.
    // Xbox Series S kontrolcüleri çok hassastır, bu yüzden 0.10 olan ölü bölgeyi
    // (Deadband)
    // 0.05'e indirerek kolun boşluğunu iyice hissettirmeden sıfırladım.
    xSpeed = edu.wpi.first.math.MathUtil.applyDeadband(xSpeed, 0.05);
    ySpeed = edu.wpi.first.math.MathUtil.applyDeadband(ySpeed, 0.05);
    azimuth = edu.wpi.first.math.MathUtil.applyDeadband(azimuth, 0.05);

    // Karesel (Quadratic) Sürüş Eğrisi: F310 gibi kollar lineerdir. Ufak dokununca
    // çok tepki verirler.
    // Değerlerin karesini alarak küçük dokunuşları ufacık hızlara (örn: 0.2 ->
    // 0.04),
    // ama tam gazı yine tam güce (1.0 -> 1.0) çeviriyoruz.
    // Robot pamuk gibi kalkacak ve duracak.
    xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
    ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);
    azimuth = Math.copySign(azimuth * azimuth * azimuth, azimuth); // Dönüşte çok ufak hareketleri emmek için küp
                                                                   // kullanılır.

    xSpeed = xLimiter.calculate(xSpeed) * Constants.ModuleConstants.LinearMaxSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * Constants.ModuleConstants.LinearMaxSpeed;
    azimuth = azimuthLimiter.calculate(azimuth) * Constants.ModuleConstants.AngularMaxSpeed;

    ChassisSpeeds speed;

    if (fieldOrientedSupplier.getAsBoolean()) {
      speed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, azimuth, swerveSubsystem.getPose().getRotation());
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

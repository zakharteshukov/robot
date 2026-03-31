package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.robot.Conversions;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldZones;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class CalculateFromDistanceCmd extends Command {
  FlyWheelSubsystem flyWheelSubsystem;
  HoodSubsystem hoodSubsystem;
  FeederSubsystem feederSubsystem;
  HopperSubsystem hopperSubsystem;
  SwerveSubsystem swerveSubsystem;
  DoubleSupplier translationX;
  DoubleSupplier translationY;
  PIDController targetAnglePID;

  public CalculateFromDistanceCmd(FlyWheelSubsystem flyWheelSubsystem, HoodSubsystem hoodSubsystem,
      FeederSubsystem feederSubsystem, HopperSubsystem hopperSubsystem,
      SwerveSubsystem swerveSubsystem,
      DoubleSupplier translationX, DoubleSupplier translationY) {
    this.flyWheelSubsystem = flyWheelSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.hopperSubsystem = hopperSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.translationX = translationX;
    this.translationY = translationY;

    targetAnglePID = new PIDController(Constants.FlyWheelConstants.targetAnglekP,
        Constants.FlyWheelConstants.targetAnglekI,
        Constants.FlyWheelConstants.targetAnglekD);

    targetAnglePID.enableContinuousInput(-180, 180);
    targetAnglePID.setTolerance(2.0);
    // targetAnglePID.setIntegratorRange(0, 0);
    addRequirements(flyWheelSubsystem, hoodSubsystem,
        feederSubsystem, hopperSubsystem,
        swerveSubsystem);
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d hubPose = FieldZones.HubZones.getHub().toTranslation2d();
    // Odometri her zaman Mavi Orjinli (Blue Origin) olduğu için Atis acisi sadece
    // iki nokta arasindaki acıdır. rotateBy 180'e gerek yok!
    Rotation2d targetAngle = hubPose.minus(swerveSubsystem.getPose().getTranslation()).getAngle();

    // PID hesabını gerçek Odometri açısıyla yap
    double azimuth = targetAnglePID.calculate(swerveSubsystem.getPose().getRotation().getDegrees(),
        targetAngle.getDegrees());

    // Sürüş eksenlerini Joystick'ten çek
    double xSpeed = translationX.getAsDouble();
    double ySpeed = translationY.getAsDouble();

    var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
    // Normal sürüşteki gibi Deadband ve Joystick Hassasiyet Ayarları
    xSpeed = edu.wpi.first.math.MathUtil.applyDeadband(xSpeed, 0.05);
    ySpeed = edu.wpi.first.math.MathUtil.applyDeadband(ySpeed, 0.05);
    xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
    ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);

    // Mavi İttifak için İleri İtildiğinde Mavi Duvara deðil Kırmızı Duvara gitmesi
    // için eksenleri tersle:
    if (!isRed) {
      xSpeed = -xSpeed;
      ySpeed = -ySpeed;
    }

    // Limitörleri uygula (Teleop ile aynı hissi yakalamak için)
    xSpeed = xSpeed * Constants.ModuleConstants.LinearMaxSpeed;
    ySpeed = ySpeed * Constants.ModuleConstants.LinearMaxSpeed;

    ChassisSpeeds speed;
    // FieldRelative Sürüş
    speed = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed,
        ySpeed,
        azimuth,
        swerveSubsystem.getPose().getRotation());

    // Şase Tekerleklerine komut yolla (Hedefe dön ve sür)
    SwerveModuleState[] states = Constants.SwerveDriveSubsystemConstants.kinematics.toSwerveModuleStates(speed);
    swerveSubsystem.setState(states);

    // Sadece hedef menzilini hesapla ve Shooter'a(Flywheel) hızı yolla.
    // Robot daha hizalanırken bile tekerleklerin dönmeye (rev up) başlaması atış hızınızı artıracaktır.
    double distance = hubPose.getDistance(swerveSubsystem.getPose().getTranslation());
    AngularVelocity flywheelSpeed = flyWheelSubsystem.calculateRpm(distance);
    flyWheelSubsystem.setFlywheelSpeed(flywheelSpeed);

    // EKRANA YAZDIRMA (DEBUG)
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("A_Mesafe", distance);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("A_HedefRPM", flywheelSpeed.in(RPM));
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("A_AciTamamMi", targetAnglePID.atSetpoint());
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("A_MevcutAciSalinimi", targetAnglePID.getPositionError());

    // Eğer Kilitlenme (Açı) Hedefe Ulaştıysa ve Shooter Hızı da yeterince yakınsa Feeder'ı Çalıştır!
    // Toleransı (50 -> 150 RPM) genişlettim ki ufak dalgalanmalarda feeder takılmasın.
    if (targetAnglePID.atSetpoint()) {
      if (Conversions.epsilonEquals(flyWheelSubsystem.getFlywheelSpeed().in(RPM), flywheelSpeed.in(RPM), 150)) {
        feederSubsystem.FeedMotorSet(-0.6); // Ateş!
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("A_AtesZamani", true);
      } else {
        feederSubsystem.FeedMotorSet(0.0); 
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("A_AtesZamani", false);
      }
    } else {
      feederSubsystem.FeedMotorSet(0.0);
      edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("A_AtesZamani", false);
    }
  }

  // Komut bittiğinde (A tuşundan elinizi çektiğinizde)
  @Override
  public void end(boolean interrupted) {
    flyWheelSubsystem.setFlywheelSpeed(RPM.of(0));
    feederSubsystem.FeedMotorSet(0.0);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("A_AtesZamani", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

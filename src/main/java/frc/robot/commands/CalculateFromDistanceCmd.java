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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  
  public CalculateFromDistanceCmd
   (FlyWheelSubsystem flyWheelSubsystem, HoodSubsystem hoodSubsystem, 
   FeederSubsystem feederSubsystem, HopperSubsystem hopperSubsystem, 
   SwerveSubsystem swerveSubsystem,
   DoubleSupplier translationX, DoubleSupplier translationY)
   {
    this.flyWheelSubsystem = flyWheelSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.hopperSubsystem = hopperSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.translationX = translationX;
    this.translationY = translationY;
    
    targetAnglePID = new PIDController
    (Constants.FlyWheelConstants.targetAnglekP, 
    Constants.FlyWheelConstants.targetAnglekI, 
    Constants.FlyWheelConstants.targetAnglekD);

    targetAnglePID.enableContinuousInput(-180, 180);
    targetAnglePID.setTolerance(2.0);
    //targetAnglePID.setIntegratorRange(0, 0);
    addRequirements
     (flyWheelSubsystem, hoodSubsystem, 
     feederSubsystem, hopperSubsystem, 
     swerveSubsystem
     );
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d hubPose = FieldZones.HubZones.getHub().toTranslation2d();
    Rotation2d targetAngle = hubPose.minus(swerveSubsystem.getPose().getTranslation())
    .getAngle().rotateBy(FieldZones.getAlliance() == Alliance.Red ? Rotation2d.kZero : Rotation2d.k180deg);
    
    // YENİ: PID hesabını direkt ham gyroya göre YAPMAMALIYIZ! 
    // Odometri (Start tuşuyla ofsetlenmiş, Limelight ile senkron) yönünü kullanmalıyız. Yoksa kendi etrafında döner veya sağa sola kayar.
    double azimuth = targetAnglePID.calculate(swerveSubsystem.getPose().getRotation().getDegrees(), targetAngle.getDegrees());

    ChassisSpeeds speed; 
    // YENİ: Robot auto-aim yaparken İleri-Geri hareketin sahaya (Sürücüye) göre olması için FieldRelativeSpeeds kullanılmalıdır.
    speed = ChassisSpeeds.fromFieldRelativeSpeeds(
      translationX.getAsDouble(),
      translationY.getAsDouble(), 
      azimuth, 
      swerveSubsystem.getPose().getRotation() // YENİ: Sürüş yönü de gerçek Odometriye bağlandı
    );

    SwerveModuleState[] states = Constants.SwerveDriveSubsystemConstants.kinematics.toSwerveModuleStates(speed);
    swerveSubsystem.setState(states);
    
    if(targetAnglePID.atSetpoint()){
      AngularVelocity flywheelSpeed = flyWheelSubsystem.calculateRpm(hubPose.getDistance(swerveSubsystem.getPose().getTranslation()));
      Angle hoodAngle = hoodSubsystem.calculateAngle(hubPose.getDistance(swerveSubsystem.getPose().getTranslation()));
      flyWheelSubsystem.setFlywheelSpeed(flywheelSpeed);
      hoodSubsystem.setHoodAngle(hoodAngle);
      if (Conversions.epsilonEquals(flyWheelSubsystem.getFlywheelSpeed().in(RPM), flywheelSpeed.in(RPM), 50) 
          && Conversions.epsilonEquals(hoodSubsystem.getHoodAngle().in(Degrees), hoodAngle.in(Degrees), 2.0))
        {
        feederSubsystem.FeedMotorSet(-0.6);
        hopperSubsystem.HopperMotorSet(-0.6);  
      }
    }
    System.out.println("targetAngle" + azimuth + "|||||" + "targetanglePid" + targetAnglePID);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //flyWheelSubsystem.setFlywheelSpeed(RPM.of(-1000));
    //hoodSubsystem.setHoodAngle(Degrees.of(0)); //TODO min hood montaj açısı 

    feederSubsystem.FeedMotorSet(0.0);
    hopperSubsystem.HopperMotorSet(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

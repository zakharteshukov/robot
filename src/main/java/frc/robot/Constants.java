package frc.robot;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class Constants {
  
  public static String limelight_name = "limelight-top";

  public static AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  

  public static final class FlyWheelConstants{
    public static int ShooterMotorLeftID = 30;
    public static int ShooterMotorRightID = 31;

    public static double kP = 0.0001;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double kS = 0.01;
    public static double kV = 0.0024;
    public static double kA = 0.0;

    public static double targetAnglekP = 0.06;
    public static double targetAnglekI = 0.0;
    public static double targetAnglekD = 0.0;

    public static double FlyWheelPosConversionF = 1.0;
    public static double FlyWheelVelConversionF = 1.0;
  }

  public static final class HoodConstants{
    public static int HoodMotorID = 32;

    public static double kP = 0.05;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double HoodPosConversionF = 10.53;
    public static double HoodVelConversionF = 10.53/60;

  }

  public static final class IntakeConstants {
    public static int IntakeMotorLeftID = 34;
    public static int IntakeMotorRightID = 35;

    public static double MinAngleDegress = 0.0; //FIXME DEGISTIR
    public static double MaxAngleDegress = 0.0;

    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.005;

    public static double IntakePosConversionF = 30.0;
    public static double IntakeVelConversionF = 0.5;
  }

  public static final class IntakeRollerConstants{
    public static int IntakeRollerID = 36;
  }

  public static final class HopperConstants {
    public static int HopperMotorID = 37;
  }

  public static final class FeederConstants{
    public static int FeederMotorID = 38;
  }

  /** Tunables for {@link frc.robot.commands.AutoBasicShootCmd}. */
  public static final class AutoConstants {
    private AutoConstants() {}

    /** Retreat distance along starting robot forward axis before shooting (meters). */
    public static final double AutonBackDistanceMeters = 1.0;
    /** Same convention as {@link frc.robot.commands.Swerve.AutoDriveSwerve}: negative = backward. */
    public static final double AutonBackXSpeedFraction = -0.5;
    /** Safety stop if distance is never reached (wheel slip, obstacle). */
    public static final double AutonBackTimeoutSeconds = 5.0;
    /** Finish line test margin for odometry (meters). */
    public static final double AutonDistanceEpsilonMeters = 0.005;

    public static final double BasicShootDurationSeconds = 5.0;
    /** Hood position while shooting (degrees). Tune to your subwoofer / line distance. */
    public static final double BasicShootHoodDegrees = 20.0;
    /** Same sign convention as teleop ({@code FeederCmd} / {@code HopperCmd}). */
    public static final double BasicShootFeeder = -0.6;
    public static final double BasicShootHopper = -0.8;
  }

  public static final class FrontLeftModule {
   public static final int DriveSparkId = 10;
   public static final int AngleSparkId = 11;
   public static final int CancoderId = 12;
   public static final double CancoderOffsetDegrees = 0 * 360;
   public static final boolean CancoderReversed = false;
   public static final boolean DriveMotorReversed = true;
   public static final boolean RotationMotorReversed = false;
  }

  public static final class FrontRightModule {
   public static final int DriveSparkId = 13;
   public static final int AngleSparkId = 14;
   public static final int CancoderId = 15;
   public static final double CancoderOffsetDegrees = 0 * 360;
   public static final boolean CancoderReversed = false;
   public static final boolean DriveMotorReversed = true;
   public static final boolean RotationMotorReversed = false;
  }
  
  public static final class BackLeftModule {
   public static final int DriveSparkId = 16;
   public static final int AngleSparkId = 17;
   public static final int CancoderId = 18;
   public static final double CancoderOffsetDegrees = 0 * 360;
   public static final boolean CancoderReversed = false;
   public static final boolean DriveMotorReversed = true;
   public static final boolean RotationMotorReversed = false;
  }

  public static final class BackRightModule {
   public static final int DriveSparkId = 19;
   public static final int AngleSparkId = 20;
   public static final int CancoderId = 21;
   public static final double CancoderOffsetDegrees = 0 * 360;
   public static final boolean CancoderReversed = false;
   public static final boolean DriveMotorReversed = true;
   public static final boolean RotationMotorReversed = false;
  }

  public static final class SwerveDriveSubsystemConstants {
    public static final double TrackWidth = Units.inchesToMeters(27.0);
    public static final double WheelBase = Units.inchesToMeters(27.0);
    //Legolas Adamdır
    
    public static final double SlewRateXLimit = 7.62;
    public static final double SlewRateYLimit = 7.62;
    public static final double SlewRateAzimuthLimit = 7.62;
    //limit = maxSpeed/time || time = maxSpeed/limit
    
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(WheelBase/2, TrackWidth/2),
      new Translation2d(WheelBase/2, -TrackWidth/2),
      new Translation2d(-WheelBase/2, TrackWidth/2),
      new Translation2d(-WheelBase/2, -TrackWidth/2)
      //FrontLeft FrontRight BackLeft BackRight
      //(X) ön:+ | arka:- (Y) sol:+ | sağ:- 
    );

    /** Max yaw rate when {@code turnSpeed == 1.0} in {@link frc.robot.commands.Swerve.AutoTurnSwerve} (rad/s). */
    public static final double AutoMaxAngularVelocityRadPerSec = 3.0;

    /** Finish turning when within this many degrees of the target (each side). */
    public static final double AutoTurnAngleToleranceDeg = 2.5;

    /**
     * Scales the commanded turn angle (like tank encoder auto). {@code 1.0} = full target;
     * use slightly below 1.0 if the robot consistently overshoots.
     */
    public static final double AutoTurnAngleScalar = 1.0;
  }
  
  public static final class ModuleConstants {
    //leblebi alalım mı?
    public static final double DriveGearRatio = 8.14 / 1;
    public static final double RotationGearRatio = 12.8 / 1;
    public static final double WheelDiameter = Units.inchesToMeters(4);
    public static final double WheelCircumference = WheelDiameter * Math.PI;

    public static final double PhysicalMaxSpeed = Units.feetToMeters(12.5); //12.5 ft/s
    // Sizin için teleop testlerindeki maksimum Sürüş ve Dönüş hızlarını %45 oranında düşürdüm. 
    // Ustalaştıkça veya yarışmaya yaklaşırken bu 7.0 rakamını yavaşça orijinali olan 12.5'a çekebilirsiniz.
    public static final double LinearMaxSpeed = Units.feetToMeters(7.0);   
    public static final double AngularMaxSpeed = Units.feetToMeters(7.0);

    public static final double DriveEncoderPositionConversionFactor = WheelCircumference/DriveGearRatio; //motor 1 şaft turu = 3.92 cm
    public static final double DriveEncoderVelocityConversionFactor = DriveEncoderPositionConversionFactor/60;
    public static final double RotationEncoderPositionConversionFactor = 360/RotationGearRatio;
    public static final double RotationEncoderVelocityConversionFactor = RotationEncoderPositionConversionFactor/60;

    public static final double kP = 0.009;
    public static final double kI = 0;
    public static final double kD = 0;

  
  }

  public final static class PathPlannerConfig {
    public static final ModuleConfig module_config = new ModuleConfig(
      Constants.ModuleConstants.WheelDiameter/2, 
      Constants.ModuleConstants.PhysicalMaxSpeed, 
      1.0, 
      DCMotor.getNEO(1),
      Constants.ModuleConstants.DriveGearRatio, 
      80, 
      1
    );
    public static final RobotConfig config = new RobotConfig(
      30,
      50,
      module_config,
      new Translation2d[]{
        new Translation2d(Constants.SwerveDriveSubsystemConstants.WheelBase/2, Constants.SwerveDriveSubsystemConstants.TrackWidth/2),
        new Translation2d(Constants.SwerveDriveSubsystemConstants.WheelBase/2, -Constants.SwerveDriveSubsystemConstants.TrackWidth/2),
        new Translation2d(-Constants.SwerveDriveSubsystemConstants.WheelBase/2, Constants.SwerveDriveSubsystemConstants.TrackWidth/2),  
        new Translation2d(-Constants.SwerveDriveSubsystemConstants.WheelBase/2, -Constants.SwerveDriveSubsystemConstants.TrackWidth/2)
      }
    );
  }





























}

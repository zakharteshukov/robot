package frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.vision.PoseEstimator;

public class SwerveSubsystem extends SubsystemBase {
  
  public SwerveModule FrontLeftModule = new SwerveModule(
  Constants.FrontLeftModule.DriveSparkId, 
  Constants.FrontLeftModule.AngleSparkId, 
  Constants.FrontLeftModule.DriveMotorReversed, 
  Constants.FrontLeftModule.RotationMotorReversed);

  public SwerveModule FrontRightModule = new SwerveModule(
  Constants.FrontRightModule.DriveSparkId, 
  Constants.FrontRightModule.AngleSparkId, 
  Constants.FrontRightModule.DriveMotorReversed, 
  Constants.FrontRightModule.RotationMotorReversed);

  public SwerveModule BackLeftModule = new SwerveModule(
  Constants.BackLeftModule.DriveSparkId,
  Constants.BackLeftModule.AngleSparkId, 
  Constants.BackLeftModule.DriveMotorReversed, 
  Constants.BackLeftModule.RotationMotorReversed);

  public SwerveModule BackRightModule = new SwerveModule(
  Constants.BackRightModule.DriveSparkId,
  Constants.BackRightModule.AngleSparkId, 
  Constants.BackRightModule.DriveMotorReversed, 
  Constants.BackRightModule.RotationMotorReversed);

  public void setState(SwerveModuleState[]states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.ModuleConstants.PhysicalMaxSpeed);
    FrontLeftModule.setDesiredState(states[0]);
    FrontRightModule.setDesiredState(states[1]);
    BackLeftModule.setDesiredState(states[2]);
    BackRightModule.setDesiredState(states[3]);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = FrontLeftModule.getState();
    states[1] = FrontRightModule.getState();
    states[2] = BackLeftModule.getState();
    states[3] = BackRightModule.getState();
    
    return states;
  }

  public ChassisSpeeds driveRelativeToRobot() { //robot oriented path
    return Constants.SwerveDriveSubsystemConstants.kinematics.toChassisSpeeds(getStates());
  }

  public void DriveRobotRelative(ChassisSpeeds robotRelativeSpeeds, DriveFeedforwards feedforwards) { //field oriented path
    var states = Constants.SwerveDriveSubsystemConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.ModuleConstants.PhysicalMaxSpeed);
    
    FrontLeftModule.setDesiredStateFeedForward(states[0], feedforwards.linearForcesNewtons()[0]);
    FrontRightModule.setDesiredStateFeedForward(states[1], feedforwards.linearForcesNewtons()[1]);
    BackLeftModule.setDesiredStateFeedForward(states[2], feedforwards.linearForcesNewtons()[2]);
    BackRightModule.setDesiredStateFeedForward(states[3], feedforwards.linearForcesNewtons()[3]);
  }
  
  public double getHeading() {
   return Math.IEEEremainder(-Constants.gyro.getAngle(), 360);  //heading = f1 - f2 * round(f1/f2)
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }
  
  public SwerveModulePosition[] getPositions () {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = FrontLeftModule.GetModulePosition();
    positions[1] = FrontRightModule.GetModulePosition();
    positions[2] = BackLeftModule.GetModulePosition();
    positions[3] = BackRightModule.GetModulePosition();
    
    return positions;
  }
  public ChassisSpeeds getSpeeds() {
    return Constants.SwerveDriveSubsystemConstants.kinematics.toChassisSpeeds(getStates());
  }

  public PoseEstimator poseEstimator = new PoseEstimator(
    Constants.SwerveDriveSubsystemConstants.kinematics, 
    this::getRotation2d, 
    this::getPositions, 
    new Pose2d(0.0,0.0, Rotation2d.fromDegrees(0)), //7 / 2
    true
  );
  
  public Pose2d getPose() {
    return poseEstimator.getPose();
  }

  public void resetPoseEstimator(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  public void stopModules() {
    FrontLeftModule.stop();
    FrontRightModule.stop();
    BackLeftModule.stop();
    BackRightModule.stop();
  }

  public void resetHeading() {
    Constants.gyro.reset();
  }

  public SwerveSubsystem() {
  
    // All other subsystem initialization
    // ...

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = Constants.PathPlannerConfig.config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPoseEstimator, // Method to reset odometry (will be called if your auto has a starting pose)
            this::driveRelativeToRobot, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> DriveRobotRelative(speeds, feedforwards), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    getRotation2d();
    getPositions();
    poseEstimator.periodic();

    // Robotun anlık hızını (X ve Y hız vektörlerinin bileşkesi) metre/saniye cinsinden buluyoruz.
    ChassisSpeeds currentSpeeds = getSpeeds();
    double currentSpeedMetersPerSec = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
    
    // Verileri Elastic Dashboard'a (SmartDashboard tablosu üzerinden) yolluyoruz
    SmartDashboard.putNumber("Robot Speed (m_s)", currentSpeedMetersPerSec);
    SmartDashboard.putNumber("Robot Heading (Deg)", getPose().getRotation().getDegrees()); // Odometri (Gyro + Vision hizalaması) yönü
  }
}

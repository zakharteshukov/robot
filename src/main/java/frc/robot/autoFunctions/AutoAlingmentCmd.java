package frc.robot.autoFunctions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldZones;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.vision.LimelightHelpers;

public class AutoAlingmentCmd extends Command {
  private final SwerveSubsystem swerve;
  private final int targetPoseIndex;
  private final PIDController xPID = new PIDController(0.05, 0.0, 0.002);
  private final PIDController yPID = new PIDController(0.05, 0.0, 0.002);
  private final PIDController azimuthPID = new PIDController(0.02, 0.0, 0.001);

  public AutoAlingmentCmd(SwerveSubsystem swerve, int targetPose) {
    this.swerve = swerve;
    this.targetPoseIndex = targetPose;
    addRequirements(swerve);
    xPID.setTolerance(0.05);
    yPID.setTolerance(0.05);
    azimuthPID.setTolerance(1.0); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    var est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.limelight_name);
    if (est == null || est.pose == null) return;

    Pose2d visionPose = est.pose;

    Pose2d targetPose = FieldZones.getZone(targetPoseIndex);

    double xCorrection = xPID.calculate(visionPose.getX(), targetPose.getX());
    double yCorrection = yPID.calculate(visionPose.getY(), targetPose.getY());
    double angleCorrection = azimuthPID.calculate(visionPose.getRotation().getDegrees(),
    targetPose.getRotation().getDegrees());

    ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(xCorrection, yCorrection, angleCorrection, swerve.getRotation2d());
    SwerveModuleState[] states = Constants.SwerveDriveSubsystemConstants.kinematics.toSwerveModuleStates(speed);
    swerve.setState(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xPID.atSetpoint() && yPID.atSetpoint() && azimuthPID.atSetpoint();
  }
}

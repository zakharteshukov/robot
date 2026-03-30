package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModuleSim extends SubsystemBase {
  private final FlywheelSim driveSim;
  private final FlywheelSim angleSim;
  private final PIDController rotPID;

  private double driveVoltage = 0.0;
  private double angleVoltage= 0.0;
  private double angleError;
  public SwerveModuleSim() {
    driveSim = new FlywheelSim(
    LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.025, 8.14),
    DCMotor.getNEO(1),
    0.0);
    angleSim = new FlywheelSim(
    LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.025, 12.8),
    DCMotor.getNEO(1),
    0.0);
    this.rotPID = new PIDController(5.0, 0.0, 0.0);
    rotPID.enableContinuousInput(-Math.PI, Math.PI);
  }
  private boolean isReversedDrive = false;
  private boolean isReversedAngle = false;
  private Rotation2d angleOffset = new Rotation2d(0);
  public void setReversedDrive(boolean reversed) {
    this.isReversedDrive = reversed;
  }
  public void setReversedAngle(boolean reversed) {
    this.isReversedAngle = reversed;
  }
  public void setAngleOffset(Rotation2d offset) {
    this.angleOffset = offset;
  }

  public void update(double dtSeconds) {
    driveSim.update(dtSeconds);
    drivePositionRad += driveSim.getAngularVelocityRadPerSec() * dtSeconds;
    angleSim.update(dtSeconds);
    anglePositionRad += angleSim.getAngularVelocityRadPerSec() * dtSeconds;
  }

  public void setDesiredState(SwerveModuleState state) {
    driveVoltage = state.speedMetersPerSecond * 12.0;
    if (isReversedDrive) {
        driveSim.setInputVoltage(-driveVoltage);
    } else {
        driveSim.setInputVoltage(driveVoltage);
    }
    
    angleError =  rotPID.calculate(anglePositionRad, state.angle.getRadians()); 
    angleVoltage = MathUtil.clamp(angleError, -12.0, 12.0);
    if (isReversedAngle) {
        angleSim.setInputVoltage(-angleVoltage);
    } else {
        angleSim.setInputVoltage(angleVoltage);
    }
  }

  private double drivePositionRad = 0.0;
  private double anglePositionRad = 0.0; 

  

  public SwerveModuleState getState() {
    double wheelRadius = Units.inchesToMeters(2); 
    double driveSpeed = driveSim.getAngularVelocityRadPerSec() * wheelRadius;
    return new SwerveModuleState(
    driveSpeed,
    Rotation2d.fromRadians(anglePositionRad).plus(angleOffset)
    );
  }

  public SwerveModulePosition getPosition() {
    double wheelRadius = Units.inchesToMeters(2); 
    double driveDistanceMeters = drivePositionRad * wheelRadius;
    return new SwerveModulePosition(
    driveDistanceMeters,
    Rotation2d.fromRadians(anglePositionRad).plus(angleOffset)
    );
  }

  @Override
  public void periodic() {

  }
}

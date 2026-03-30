package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollerSubsystem extends SubsystemBase {
  SparkMax intakeRoller;
  SparkMaxConfig intakeRollerConfig;
  public IntakeRollerSubsystem() {
    intakeRoller = new SparkMax(Constants.IntakeRollerConstants.IntakeRollerID, MotorType.kBrushless);
    intakeRollerConfig = new SparkMaxConfig();
    intakeRollerConfig.idleMode(IdleMode.kCoast);
    intakeRoller.configure(intakeRollerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters); 
  }

  public void IntakeRollerSet(double set){
    intakeRoller.set(set);
  }

  @Override
  public void periodic() {
    
  }
}

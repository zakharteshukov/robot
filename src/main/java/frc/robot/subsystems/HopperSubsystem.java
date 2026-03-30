package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.ResetMode;

public class HopperSubsystem extends SubsystemBase {
  SparkMax hopperMotor;
  SparkMaxConfig hopperConfig; 
  
  public HopperSubsystem() {
    hopperMotor = new SparkMax(Constants.HopperConstants.HopperMotorID, MotorType.kBrushless);
    hopperConfig = new SparkMaxConfig();
    hopperConfig.idleMode(IdleMode.kCoast);
    hopperMotor.configure(hopperConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  
  public void HopperMotorSet(double set){
    hopperMotor.set(set);
    
  }
  
  @Override
  public void periodic() {
    
  }
}

package frc.robot.commands;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCmd extends Command {
  IntakeSubsystem intakeSubsystem;
  double set;
  public IntakeCmd(IntakeSubsystem intakeSubsystem, double set) {
    this.intakeSubsystem = intakeSubsystem;
    this.set = set; 
    addRequirements(intakeSubsystem);
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    intakeSubsystem.setIntakeMotor(set);
  }

  
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeMotor(0);
  }

  
  @Override
  public boolean isFinished() {
     return false;
  }
}

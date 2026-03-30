// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot{
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private Telemetry telemetry;
  private Telemetry.swerveSimTelemetry swerveSim;
  private Telemetry.AutoGoToZoneSim autoGoToSim;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    DataLogManager.start();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on
    Logger.addDataReceiver(new WPILOGWriter());
    
  
    Logger.recordMetadata("odyssey2026", "official"); 
    Logger.addDataReceiver(new NT4Publisher());        
    Logger.start();
    m_robotContainer = new RobotContainer();
    telemetry = new Telemetry();
    if(RobotBase.isSimulation()){
      swerveSim = telemetry.new swerveSimTelemetry();
      autoGoToSim = telemetry.new AutoGoToZoneSim(swerveSim);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    telemetry.swerveRealTelemetry(m_robotContainer.swerveSubsystem);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}
  private final CommandXboxController driverController = new CommandXboxController(4); 
  private double applyDeadband(double value) {
    return Math.abs(value) < 0.05 ? 0.0 : value;
  }
  
  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    double xSpeed = applyDeadband(driverController.getRawAxis(1)); 
    double ySpeed = applyDeadband(driverController.getRawAxis(0));  
    double rot    = applyDeadband(driverController.getRawAxis(3));

    driverController.leftBumper().onTrue(new InstantCommand(()-> swerveSim.resetHeading()));
    driverController.x().onTrue(autoGoToSim.goTo(FieldZones.getZone(1)));
    driverController.y().onTrue(autoGoToSim.goTo(FieldZones.getZone(0)));
    driverController.b().onTrue(autoGoToSim.goTo(FieldZones.getZone(2)));
    
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, rot, swerveSim.getRotation2d());
    SwerveModuleState[] states = Constants.SwerveDriveSubsystemConstants.kinematics.toSwerveModuleStates(speeds);
    swerveSim.setState(states);
    swerveSim.periodic();
    SmartDashboard.putNumber("Omega", speeds.omegaRadiansPerSecond);
  }
}

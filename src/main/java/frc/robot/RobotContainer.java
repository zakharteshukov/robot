package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autoFunctions.AutoAlingmentCmd;
import frc.robot.autoFunctions.AutoGoToZone;
import frc.robot.commands.HopperCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.IntakeRollerCmd;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoBasicShootCmd;
import frc.robot.commands.Swerve.AutoDriveRobotRelativeDistance;
import frc.robot.commands.CalculateFromDistanceCmd;
import frc.robot.commands.FeederCmd;
import frc.robot.commands.Swerve.PadWhithDriveCmd;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.vision.PoseEstimator;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Command defaultAutonomousCommand;
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  HopperSubsystem hopperSubsystem = new HopperSubsystem();
  IntakeRollerSubsystem intakeRollerSubsystem = new IntakeRollerSubsystem();
  FeederSubsystem feederSubsystem = new FeederSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  HoodSubsystem hoodSubsystem = new HoodSubsystem();
  FlyWheelSubsystem flyWheelSubsystem = new FlyWheelSubsystem();
  AutoGoToZone GoTo = new AutoGoToZone(swerveSubsystem);
  CommandXboxController pad = new CommandXboxController(0);
  CommandXboxController pad2 = new CommandXboxController(1);
  double rateRPM = 0.0;
  double rateAngle = 0.0; // 20.52994

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // registerNamedCommands();
    defaultAutonomousCommand =
        new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new AutoDriveRobotRelativeDistance(
                    swerveSubsystem,
                    AutoConstants.AutonBackXSpeedFraction,
                    AutoConstants.AutonBackDistanceMeters,
                    AutoConstants.AutonBackTimeoutSeconds),
                Commands.run(
                    () ->
                        AutoBasicShootCmd.applyHubTrackedAim(
                            swerveSubsystem, flyWheelSubsystem, hoodSubsystem),
                    flyWheelSubsystem,
                    hoodSubsystem)),
            new AutoBasicShootCmd(
                flyWheelSubsystem,
                hoodSubsystem,
                feederSubsystem,
                hopperSubsystem,
                swerveSubsystem,
                AutoConstants.BasicShootDurationSeconds));
    swerveSubsystem.setDefaultCommand(new PadWhithDriveCmd(() -> pad.getLeftY(), () -> pad.getLeftX(),
        () -> pad.getRightX(), () -> true, swerveSubsystem));
    configureBindings();
  }

  /*
   * private void registerNamedCommands() { //TODO BUNLARIN HEPSİNİN + -
   * DEĞERİNDEN DİKKAT EDİLEREK EMİN OLUNACAK
   * NamedCommands.registerCommand("HopperForward", new HopperCmd(hopperSubsystem,
   * 0.8));
   * NamedCommands.registerCommand("HopperReverse", new HopperCmd(hopperSubsystem,
   * -0.8));
   * NamedCommands.registerCommand("HopperStop", new HopperCmd(hopperSubsystem,
   * 0));
   * 
   * NamedCommands.registerCommand("IntakeRollerForward", new
   * IntakeRollerCmd(intakeRollerSubsystem, 0.8));
   * NamedCommands.registerCommand("IntakeRollerReverse", new
   * IntakeRollerCmd(intakeRollerSubsystem, -0.8));
   * NamedCommands.registerCommand("IntakeRollerStop", new
   * IntakeRollerCmd(intakeRollerSubsystem, 0));
   */

  // NamedCommands.registerCommand("FeederBaslat", new FeederCmd(feederSubsystem,
  // 0,8));

  /*
   * NamedCommands.registerCommand("FeederReverse", new FeederCmd(feederSubsystem,
   * -0.8));
   * NamedCommands.registerCommand("FeederStop", new FeederCmd(feederSubsystem,
   * 0));
   * 
   * NamedCommands.registerCommand("IntakeDown", new IntakeCmd(intakeSubsystem,
   * 90.0));
   * NamedCommands.registerCommand("IntakeUp", new IntakeCmd(intakeSubsystem,
   * 0.0));
   * 
   * 
   * NamedCommands.registerCommand("ShooterOn", new CalculateFromDistanceCmd(
   * flyWheelSubsystem, hoodSubsystem, feederSubsystem,
   * hopperSubsystem, swerveSubsystem, () -> 0.0, () -> 0.0));
   * //Daha mantıklı bir şey düşünülebilir
   * NamedCommands.registerCommand("ShooterOff", new InstantCommand(() -> {
   * feederSubsystem.FeedMotorSet(0);
   * hopperSubsystem.HopperMotorSet(0);
   * },
   * 
   * feederSubsystem, hopperSubsystem));
   * 
   * }
   */

  private void configureBindings() {
    pad.y().toggleOnTrue(new IntakeRollerCmd(intakeRollerSubsystem, 0.4));
    pad.b().toggleOnTrue(new FeederCmd(feederSubsystem, -0.6));
    pad.x().toggleOnTrue(new HopperCmd(hopperSubsystem, -0.8));

    /// pad.leftBumper().toggleOnTrue(new IntakeCmd(intakeSubsystem, -100));
    // pad2.rightBumper().whileTrue(new IntakeCmd(intakeSubsystem, 1));
    // pad2.leftBumper().whileTrue(new IntakeCmd(intakeSubsystem, -1));
    // pad2.rightTrigger().whileTrue(new HopperCmd(hopperSubsystem, 1));
    // pad2.leftTrigger().whileTrue(new HopperCmd(hopperSubsystem, -1));

    // pad.leftTrigger().whileTrue(Commands.run(() ->
    // intakeSubsystem.incrementTargetAngle(0.9), intakeSubsystem));
    /// pad.rightTrigger().whileTrue(Commands.run(() ->
    // intakeSubsystem.incrementTargetAngle(-0.9), intakeSubsystem));
    pad.povUp().whileTrue(new InstantCommand(() -> {
      rateAngle += 1;
    }));
    pad.povDown().whileTrue(new InstantCommand(() -> {
      rateAngle -= 1;
    }));
    pad.povRight().whileTrue(new InstantCommand(() -> {
      rateRPM -= 50;
    }));
    pad.povLeft().whileTrue(new InstantCommand(() -> {
      rateRPM += 50;
    })); // HASAS AYARİÇİN 50 YE DÜŞÜRDM

    // RobotContainer.java içinde

    pad.rightBumper().onTrue(new RunCommand(() -> hoodSubsystem.setHoodAngle(Degrees.of(rateAngle)), hoodSubsystem));
    pad.leftBumper()
        .onTrue(new RunCommand(() -> flyWheelSubsystem.setFlywheelSpeed(RPM.of(rateRPM)), flyWheelSubsystem));

    // pad.a().whileTrue(new FeederCmd(feederSubsystem, -0.6).alongWith(new
    // HopperCmd(hopperSubsystem, -0.6)));
    pad.a().whileTrue(new CalculateFromDistanceCmd(flyWheelSubsystem, hoodSubsystem,
        swerveSubsystem, () -> pad.getLeftY(), () -> pad.getLeftX()));
    pad.rightTrigger().whileTrue(new IntakeCmd(intakeSubsystem, 0.8));
    pad.leftTrigger().whileTrue(new IntakeCmd(intakeSubsystem, -0.8));

    // YENİ: Intake Ağır Olduğu İçin "Yerçekimi Telafili" Otomatik Kaldır-İndir
    // Döngüsü
    // Kaldırmak zor olduğu için gücü 0.2 (%20) veriyoruz.
    // İndirirken yerçekimi yardım edip kendi saldığı için sadece -0.05 veriyoruz
    // (çarpmasın diye).
    pad.leftStick().toggleOnTrue(
        new edu.wpi.first.wpilibj2.command.SequentialCommandGroup(
            new IntakeCmd(intakeSubsystem, 0.7).withTimeout(0.3), // Sıkıca Kaldır
            new IntakeCmd(intakeSubsystem, -0.1).withTimeout(0.4) // Yerçekimiyle beraber hafifçe sal
        ).repeatedly());

    pad.rightStick().toggleOnTrue(new IntakeRollerCmd(intakeRollerSubsystem, -0.4));

    // pad.x().onTrue(new InstantCommand(()->swerveSubsystem.resetHeading(),
    //
    // swerveSubsystem));

    //
    // YENİ: İttifak Yönüne Göre Akıllı Gyro Sıfırlama (Pusula Sıfırlama)
    pad.start().onTrue(new InstantCommand(() -> {
      var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
      boolean isRed = alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
      // Kırmızıysak sırtımız 180'e bakar, Maviysek sırtımız 0'a bakar.
      double resetAngle = isRed ? 180.0 : 0.0;
      swerveSubsystem.resetPoseEstimator(new edu.wpi.first.math.geometry.Pose2d(
          swerveSubsystem.getPose().getTranslation(),
          edu.wpi.first.math.geometry.Rotation2d.fromDegrees(resetAngle)));
    }));

    // pad.povUpLeft().onTrue(new InstantCommand(()->PoseEstimator.visionEnable =
    // !PoseEstimator.visionEnable));
    /*
     * pad.a().onTrue(new SequentialCommandGroup(
     * new AutoGoToZone(swerveSubsystem).goTo(FieldZones.getZone(0)),
     * new AutoAlingmentCmd(swerveSubsystem, 0),
     * new ParallelCommandGroup(
     * ))
     * );
     */

    /*
     * pad.x().onTrue(new SequentialCommandGroup(
     * new AutoGoToZone(swerveSubsystem).goTo(FieldZones.getZone(1)),
     * new AutoAlingmentCmd(swerveSubsystem, 1),
     * new ParallelCommandGroup(
     * new FlyWheelCmd(FlyWheelSubsystem, 1),
     * new SequentialCommandGroup(
     * new WaitCommand(1),
     * new HopperCmd(hopperSubsystem, 1)
     * )
     * ))
     * );
     * 
     * pad.b().onTrue(new SequentialCommandGroup(
     * new AutoGoToZone(swerveSubsystem).goTo(FieldZones.getZone(2)),
     * new AutoAlingmentCmd(swerveSubsystem, 2),
     * new ParallelCommandGroup(
     * new FlyWheelCmd(FlyWheelSubsystem, 1),
     * new SequentialCommandGroup(
     * new WaitCommand(1),
     * new HopperCmd(hopperSubsystem, 1)
     * )
     * ))
     * );
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return Drive back {@link AutoConstants#AutonBackDistanceMeters} m (odometry), spinning up
   *     shooter during the move, then {@link AutoBasicShootCmd} for {@link
   *     AutoConstants#BasicShootDurationSeconds} s; no dashboard chooser.
   */
  public Command getAutonomousCommand() {
    return defaultAutonomousCommand;
  }
}

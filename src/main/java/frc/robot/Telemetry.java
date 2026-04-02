package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve.SwerveModuleSim;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.vision.PoseEstimator;

public class Telemetry {
    SwerveSubsystem swerveSubsystem;
    
    public void swerveRealTelemetry(SwerveSubsystem swerveSubsystemReal){
        this.swerveSubsystem = swerveSubsystemReal;
        
        Logger.recordOutput("getStates", swerveSubsystem.getStates());
        Logger.recordOutput("getSpeeds", swerveSubsystem.getSpeeds());
        Logger.recordOutput("getRotation2d", swerveSubsystem.getRotation2d());
        Logger.recordOutput("getPose", swerveSubsystem.getPose());

        SmartDashboard.putNumber("FrontLeft Drive Encoder", swerveSubsystem.FrontLeftModule.GetDrivePosition());
        SmartDashboard.putNumber("FrontRight Drive Encoder", swerveSubsystem.FrontRightModule.GetDrivePosition());
        
        SmartDashboard.putNumber("BackLeft Drive Encoder", swerveSubsystem.BackLeftModule.GetDrivePosition());
        SmartDashboard.putNumber("BackRight Drive Encoder", swerveSubsystem.BackRightModule.GetDrivePosition());

        SmartDashboard.putNumber("FrontLeft Rotation Encoder", swerveSubsystem.FrontLeftModule.GetRotationPosition());
        SmartDashboard.putNumber("FrontRight Rotation Encoder", swerveSubsystem.FrontRightModule.GetRotationPosition());
        
        SmartDashboard.putNumber("BackLeft Rotation Encoder", swerveSubsystem.BackLeftModule.GetRotationPosition());
        SmartDashboard.putNumber("BackRight Rotation Encoder", swerveSubsystem.BackRightModule.GetRotationPosition());

        SmartDashboard.putNumber("angle ", swerveSubsystem.getHeading());
        SmartDashboard.putNumber("Odometry X", swerveSubsystem.poseEstimator.getPose().getX());
        SmartDashboard.putNumber("Odometry Y", swerveSubsystem.poseEstimator.getPose().getY());

        Pose2d currentRobot = swerveSubsystem.getPose();
        Translation2d BlueHubCenter = new Translation2d(4.618, 4.04); //Pathplanner bakrak degistirdim 3.08den 4.618 cekiyorum !!DEGİSEBİLİR
        Translation2d RedHubCenter = new Translation2d(11.907 , 4.04); //Degistirdim  13.46 girilmisti pathplanner bakarak 11.907 çekilfr !!!DEGİSEBİLİR
        double BlueHubToDistance = currentRobot.getTranslation().getDistance(BlueHubCenter);
        double RedHubToDistance = currentRobot.getTranslation().getDistance(RedHubCenter);

        SmartDashboard.putNumber("BlueHubToDistance", BlueHubToDistance);
        SmartDashboard.putNumber("RedHubToDistance", RedHubToDistance);

        /*double distanceHub = Math.sqrt(
            Math.pow(12.0, swerveSubsystem.poseEstimator.getPose().getX()) + Math.pow(5.95, swerveSubsystem.poseEstimator.getPose().getY())
        );
        SmartDashboard.putNumber("hubuzaklik", distanceHub);*/
    }

    public class swerveSimTelemetry extends SubsystemBase{
        SwerveModuleSim frontleftSim = new SwerveModuleSim(); 
        SwerveModuleSim frontrightSim = new SwerveModuleSim(); 
        SwerveModuleSim backleftSim = new SwerveModuleSim(); 
        SwerveModuleSim backrightSim = new SwerveModuleSim(); 
        
        public ChassisSpeeds driveRelativeToRobot() {
            return Constants.SwerveDriveSubsystemConstants.kinematics.toChassisSpeeds(getStates());
        }

        public void DriveRobotRelative(ChassisSpeeds robotRelativeSpeeds) { 
            var states = Constants.SwerveDriveSubsystemConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.ModuleConstants.PhysicalMaxSpeed);
            setState(states);
        }

        public void resetPoseEstimator(Pose2d pose) {
            poseEstimatorSim.resetPose(pose);
        }

        public Pose2d getPose(){
            return poseEstimatorSim.getPose();
        }

        public swerveSimTelemetry(){
            //frontrightSim.setAngleOffset(Rotation2d.fromDegrees(0));

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
                (speeds, feedforwards) -> DriveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
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

        public void setState(SwerveModuleState[]states) {
            SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.ModuleConstants.PhysicalMaxSpeed);
            frontleftSim.setDesiredState(states[0]);
            frontrightSim.setDesiredState(states[1]);
            backleftSim.setDesiredState(states[2]);
            backrightSim.setDesiredState(states[3]);
        }

        public SwerveModuleState[] getStates() {
            SwerveModuleState[] states = new SwerveModuleState[4];
            states[0] = frontleftSim.getState();
            states[1] = frontrightSim.getState();
            states[2] = backleftSim.getState();
            states[3] = backrightSim.getState();
            return states;
        }
        public SwerveModulePosition[] getPositions() {
            SwerveModulePosition[] positions = new SwerveModulePosition[4];
            positions[0] = frontleftSim.getPosition();
            positions[1] = frontrightSim.getPosition();
            positions[2] = backleftSim.getPosition();
            positions[3] = backrightSim.getPosition();
            return positions;
        }
        
        PoseEstimator poseEstimatorSim = new PoseEstimator(Constants.SwerveDriveSubsystemConstants.kinematics, 
        this::getRotation2d,
        this::getPositions, 
        new Pose2d(1.0,5.0, new Rotation2d(0)),
        false);

        private double simYawRadians = 0.0;

        public Rotation2d getRotation2d(){
            return new Rotation2d(simYawRadians);
        }

        public void resetHeading(){
            simYawRadians = 0;
        }

        public void periodic(){
            frontleftSim.update(0.02);
            frontrightSim.update(0.02);
            backleftSim.update(0.02);
            backrightSim.update(0.02);

            SwerveModuleState[] currentStates = new SwerveModuleState[]{
                frontleftSim.getState(),
                frontrightSim.getState(),
                backleftSim.getState(),
                backrightSim.getState()
            };
            
            var actualSpeeds = Constants.SwerveDriveSubsystemConstants.kinematics.toChassisSpeeds(currentStates);
            simYawRadians -= actualSpeeds.omegaRadiansPerSecond * 0.02;
            poseEstimatorSim.periodic();
            getPose();

            Logger.recordOutput("getStatesSim", getStates());
            Logger.recordOutput("getSpeedsSim", Constants.SwerveDriveSubsystemConstants.kinematics.toChassisSpeeds(
                frontleftSim.getState(),
                frontrightSim.getState(),
                backleftSim.getState(),
                backrightSim.getState()
            ));
            SmartDashboard.putNumber("SimYawRadians", simYawRadians);
            Logger.recordOutput("getRotation2dSim", getRotation2d());
            Logger.recordOutput("getPoseSim", getPose());
            SmartDashboard.putString("Pose", getPose().toString());
        }
    }

    
    public class AutoGoToZoneSim{
        swerveSimTelemetry swerveSim;
        public AutoGoToZoneSim(swerveSimTelemetry swerveSim){
        this.swerveSim = swerveSim;
        }
    
        private PathConstraints getConstraints(double distanceMeters) {
            if (distanceMeters > 5.0) {
                return new PathConstraints(
                4.0, 
                3.5, 
                10.0, 
                12.0, 
                12.0, 
                false
                );
            } 
            else {
                return new PathConstraints(
                2.5, 
                2.0, 
                6.0, 
                8.0, 
                12.0, 
                false
                );
            }
        }
    
        public Command goTo(Pose2d targetPose) {
            double distance = swerveSim.getPose()
            .getTranslation()
            .getDistance(targetPose.getTranslation());
            return AutoBuilder.pathfindToPose(
                targetPose,
                getConstraints(distance),
                0.0 
            );
        }

        public void periodic(){
            //Logger.recordOutput("", null);
        }
    }
    
}

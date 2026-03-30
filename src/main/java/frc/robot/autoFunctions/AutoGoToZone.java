package frc.robot.autoFunctions;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class AutoGoToZone {
    SwerveSubsystem swerve;
    public AutoGoToZone(SwerveSubsystem swerve){
        this.swerve = swerve;
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
        double distance = swerve.getPose()
        .getTranslation()
        .getDistance(targetPose.getTranslation());
        return AutoBuilder.pathfindToPose(
            targetPose,
            getConstraints(distance),
            0.0 
        );
    }
}

package frc.robot;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldZones {

    public static Alliance getAlliance(){
        Alliance getAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return getAlliance;
    }

    public static final Pose2d[] BLUE_ZONES_SHOOT = {
        new Pose2d(3.0, 3.0, Rotation2d.fromDegrees(45)),
        new Pose2d(2.5, 4.0, Rotation2d.fromDegrees(0)),
        new Pose2d(3.0, 5.0, Rotation2d.fromDegrees(-45))
    };

    public static final Pose2d[] RED_ZONES_SHOOT = {
        new Pose2d(14.5, 3.0, Rotation2d.fromDegrees(135)),
        new Pose2d(14.0, 4.0, Rotation2d.fromDegrees(180)),
        new Pose2d(14.5, 5.0, Rotation2d.fromDegrees(-135))
    };
    
    public static Pose2d getZone(int targetPose) {
        return (getAlliance() == Alliance.Blue) ? BLUE_ZONES_SHOOT[targetPose] : RED_ZONES_SHOOT[targetPose];
    }

    public static class HubZones{
        public static final Distance FIELD_LENGTH = Inches.of(650.12);
        public static final Distance FIELD_WIDTH = Inches.of(316.64);

        public static final Translation3d HUB_BLUE 
            = new Translation3d(Inches.of(181.56), FIELD_WIDTH.div(2), Inches.of(56.4));
        public static final Translation3d HUB_RED 
            = new Translation3d(FIELD_LENGTH.minus(Inches.of(181.56)), FIELD_WIDTH.div(2), Inches.of(56.4));
        
        public static Translation3d getHub(){
            return (getAlliance() == Alliance.Blue) ? HUB_BLUE : HUB_RED;
        }
    }
    
}

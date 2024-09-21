package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class Mid5Piece extends SequentialCommandGroup{
    public Mid5Piece(SwerveDrivetrain swerve, List<PathPlannerPath>pathGroup, SuperSystem supersystem){
        Pose2d startingPose = new Pose2d(1.33,5.55, new Rotation2d());
        addCommands(
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            Commands.sequence(
                // Preload
                Commands.deadline(
                    Commands.waitSeconds(1.5),
                    supersystem.shootSpeaker()
                    // get Piece out robot
                ),
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(0)),
                    supersystem.intakeNote()
                    // Piece 1 a01Path
                ),
                Commands.deadline(
                    Commands.waitSeconds(1.5),
                    supersystem.shootSpeaker()
                    // get Piece out robot
                ),
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(1)),
                    supersystem.intakeNote()
                    // Piece 2 a02Path
                ),
                Commands.deadline(
                    Commands.waitSeconds(1.5),
                    supersystem.shootSpeaker()
                    // get Piece out robot
                ),
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(2)),
                    supersystem.intakeNote()
                    // Piece 3 a03Path
                ),
                Commands.deadline(
                    Commands.waitSeconds(1.5),
                    supersystem.shootSpeaker()
                    // get Piece out robot
                ),
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(3)),
                    supersystem.intakeNote()
                    // Piece 4 c37Path
                ),
                
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(4))
                    // Return to a position in which the robot can shoot
                ),
                Commands.deadline(
                    Commands.waitSeconds(1.5),
                    supersystem.shootSpeaker()
                    // get Piece out robot
                )
                
            )
            
        );
    }
}
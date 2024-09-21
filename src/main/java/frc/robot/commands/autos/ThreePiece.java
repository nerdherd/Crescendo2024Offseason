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


public class ThreePiece extends SequentialCommandGroup {
    public ThreePiece(SwerveDrivetrain swerve, List<PathPlannerPath> autoPath,SuperSystem superSystem){
        Pose2d startingPose = new Pose2d(1.33,5.55, new Rotation2d());

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.sequence(
                // Preload
                Commands.deadline(
                    Commands.waitSeconds(1.5),
                    superSystem.shootSpeaker() // change later
                ),
                // Second piece
                Commands.deadline(
                    AutoBuilder.followPath(autoPath.get(0)),
                    superSystem.intakeNew()
                ),
                // Shoot it
                Commands.deadline(
                    Commands.waitSeconds(1),
                    superSystem.shootSpeaker()
                ),
                // Go to Third piece
                Commands.deadline(
                    AutoBuilder.followPath(autoPath.get(1)),
                    superSystem.intakeNew()
                ),
                // Shoot it
                Commands.deadline(
                    Commands.waitSeconds(1),
                    superSystem.shootSpeaker()
                )
            )
        );
    }
}
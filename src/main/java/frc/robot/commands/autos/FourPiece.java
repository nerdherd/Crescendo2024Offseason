package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class FourPiece extends SequentialCommandGroup {
    public FourPiece(SwerveDrivetrain swerve, String autoPath,SuperSystem superSystem){
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(() -> swerve.resetOdometry(startingPose)),
            Commands.sequence(
                // Preload
                Commands.deadline(
                    Commands.waitSeconds(1.5),
                    superSystem.shootSpeaker() // change later
                ),
                // Second piece
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(0)),
                    superSystem.intakeNew()
                ),
                // Shoot it
                Commands.deadline(
                    Commands.waitSeconds(1),
                    superSystem.shootSpeaker()
                ),
                // Go to Third piece
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(1)),
                    superSystem.intakeNew()
                ),
                // Shoot it
                Commands.deadline(
                    Commands.waitSeconds(1),
                    superSystem.shootSpeaker()
                ),
                // Go to Fourth piece
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(2)),
                    superSystem.intakeNew()
                ),
                // Shoot
                Commands.deadline(
                    Commands.waitSeconds(1),
                    superSystem.shootSpeaker()
                )
            )
        );
    }
}
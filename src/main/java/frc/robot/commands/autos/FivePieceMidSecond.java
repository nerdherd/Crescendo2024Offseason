package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class FivePieceMidSecond extends SequentialCommandGroup{
    public FivePieceMidSecond(SwerveDrivetrain swerve, String autoPath, Supersystem supersystem){
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);
        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees()))
            Commands.runOnce(()->swerve.resetOdometry(startingPose)),
            Commands.sequence(
                // Preload
                Commands.deadline(
                    Commands.waitSeconds(seconds:1.5),
                    supersystem.shoot()
                    // get Piece out robot
                )
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(0)),
                    supersystem.intake()
                    // Piece 1
                )
                Commands.deadline(
                    Commands.waitSeconds(seconds:1.5),
                    supersystem.shoot()
                    // get Piece out robot
                )
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(1)),
                    supersystem.intake()
                    // Piece 2
                )
                Commands.deadline(
                    Commands.waitSeconds(seconds:1.5),
                    supersystem.shoot()
                    // get Piece out robot
                )
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(2)),
                    supersystem.intake()
                    // Piece 3
                )
                Commands.deadline(
                    Commands.waitSeconds(seconds:1.5),
                    supersystem.shoot()
                    // get Piece out robot
                )
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(3)),
                    supersystem.intake()
                    // Piece 4
                )
                
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(4)),
                    // Return to a position in which the robot can shoot
                )
                Commands.deadline(
                    Commands.waitSeconds(seconds:1.5),
                    supersystem.shoot()
                    // get Piece out robot
                )
                
            )
            
        )
    }
}
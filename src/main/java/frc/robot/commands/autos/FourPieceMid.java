package frc.robot.commands.autos;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.NoteAssistance;

public class FourPieceMid extends SequentialCommandGroup {
    
    // to be tested. Do not use it before test

    public FourPieceMid(SwerveDrivetrain swerve, SuperSystem superSystem, NoteAssistance noteCamera, List<PathPlannerPath> pathGroup){
        Pose2d startingPose = new Pose2d(1.33, 5.55, new Rotation2d());//GetStartPoseInPath(pathGroup.get(0));
        addCommands(
                Commands.runOnce(swerve.getImu()::zeroAll),
                Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
                Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
                
                Commands.sequence(      
                    // Preload
                    superSystem.shooterRoller.setRightVelocityCommand(-1),
                    Commands.deadline(
                        Commands.waitUntil(() -> superSystem.shooterRoller.atTargetVelocityRight()),
                        Commands.waitSeconds(2)
                    ),
                    superSystem.shooterRoller.setLeftVelocityCommand(-1),

                    // Drive to note 1
                    Commands.parallel(
                        AutoBuilder.followPath(pathGroup.get(0)),
                        superSystem.intakeBasic()
                        // Commands.deadline(
                        //     Commands.waitSeconds(2),
                        //     Commands.waitUntil(() -> superSystem.intakeBeamBreak.noteSensed())
                        // )
                    ),

                    // shoot first note after preload
                    AutoBuilder.followPath(pathGroup.get(1)),//a10
                    // Stop note from popping out
                    superSystem.shooterRoller.setRightVelocityCommand(-1),
                    Commands.deadline(
                        Commands.waitUntil(() -> superSystem.shooterRoller.atTargetVelocityRight()),
                        Commands.waitSeconds(2)
                    ),
                    superSystem.shooterRoller.setLeftVelocityCommand(-1),


                    // Preload
                    superSystem.shooterRoller.setRightVelocityCommand(-1),
                    Commands.deadline(
                        Commands.waitUntil(() -> superSystem.shooterRoller.atTargetVelocityRight()),
                        Commands.waitSeconds(2)
                    ),
                    superSystem.shooterRoller.setLeftVelocityCommand(-1),

                    // Drive to note 2
                    Commands.parallel(
                        AutoBuilder.followPath(pathGroup.get(2)),//a02
                        superSystem.intakeBasic()
                        // Commands.deadline(
                        //     Commands.waitSeconds(2),
                        //     Commands.waitUntil(() -> superSystem.intakeBeamBreak.noteSensed())
                        // )
                    ),

                    // shoot first note after preload
                    AutoBuilder.followPath(pathGroup.get(3)),//a20
                    // Stop note from popping out
                    superSystem.shooterRoller.setRightVelocityCommand(-1),
                    Commands.deadline(
                        Commands.waitUntil(() -> superSystem.shooterRoller.atTargetVelocityRight()),
                        Commands.waitSeconds(2)
                    ),
                    superSystem.shooterRoller.setLeftVelocityCommand(-1),

                    // Preload
                    superSystem.shooterRoller.setRightVelocityCommand(-1),
                    Commands.deadline(
                        Commands.waitUntil(() -> superSystem.shooterRoller.atTargetVelocityRight()),
                        Commands.waitSeconds(2)
                    ),
                    superSystem.shooterRoller.setLeftVelocityCommand(-1),

                    // Drive to note 3
                    Commands.parallel(
                        AutoBuilder.followPath(pathGroup.get(4)),//a03
                        superSystem.intakeBasic()
                        // Commands.deadline(
                        //     Commands.waitSeconds(2),
                        //     Commands.waitUntil(() -> superSystem.intakeBeamBreak.noteSensed())
                        // )
                    ),

                    // shoot first note after preload
                    AutoBuilder.followPath(pathGroup.get(5)),//a30
                    // Stop note from popping out
                    superSystem.shooterRoller.setRightVelocityCommand(-1),
                    Commands.deadline(
                        Commands.waitUntil(() -> superSystem.shooterRoller.atTargetVelocityRight()),
                        Commands.waitSeconds(2)
                    ),
                    superSystem.shooterRoller.setLeftVelocityCommand(-1)
                )  
        );
    }
}

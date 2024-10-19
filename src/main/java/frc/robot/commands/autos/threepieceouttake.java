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
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.NoteAssistance;

public class threepieceouttake extends SequentialCommandGroup {
    
    // to be tested. Do not use it before test

    public threepieceouttake(SwerveDrivetrain swerve, IntakeRoller intakeRoller, List<PathPlannerPath> pathGroup){
        Pose2d startingPose = new Pose2d(1.33, 5.55, new Rotation2d());//GetStartPoseInPath(pathGroup.get(0));
        addCommands(
                Commands.runOnce(swerve.getImu()::zeroAll),
                Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
                Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
                intakeRoller.setEnabledCommand(true),
                Commands.sequence(      
                    // Preload
                    // Drive to note 1
                    Commands.parallel(
                        AutoBuilder.followPath(pathGroup.get(0)),
                        intakeRoller.intakeCommand()
                        // Commands.deadline(
                        //     Commands.waitSeconds(2),
                        //     Commands.waitUntil(() -> superSystem.intakeBeamBreak.noteSensed())
                        // )
                    ),

                    // shoot first note after preload
                    AutoBuilder.followPath(pathGroup.get(1)),//a10
                        intakeRoller.outtakeCommand(),

                    // Drive to note 2
                    Commands.parallel(
                        AutoBuilder.followPath(pathGroup.get(2)),
                        intakeRoller.intakeCommand()

                        // Commands.deadline(
                        //     Commands.waitSeconds(2),
                        //     Commands.waitUntil(() -> superSystem.intakeBeamBreak.noteSensed())
                        // )
                    ),

                    // shoot second note after preload
                    AutoBuilder.followPath(pathGroup.get(3)),//a20
                    intakeRoller.outtakeCommand(),

                    intakeRoller.setEnabledCommand(false)

        
                )  
        );
    }
}

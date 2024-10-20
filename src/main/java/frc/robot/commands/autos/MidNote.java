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

public class MidNote extends SequentialCommandGroup {
    
    // to be tested. Do not use it before test

    public MidNote(SwerveDrivetrain swerve, IntakeRoller intake, List<PathPlannerPath> pathGroup){
        Pose2d startingPose = new Pose2d(0, 0, new Rotation2d());//GetStartPoseInPath(pathGroup.get(0));
        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            Commands.sequence(
                AutoBuilder.followPath(pathGroup.get(0)),
                AutoBuilder.followPath(pathGroup.get(1))
            )
        ); 
        // Pose2d startingPose = new Pose2d(0.93, 2.01, new Rotation2d());//GetStartPoseInPath(pathGroup.get(0));
        // addCommands(
        //     Commands.runOnce(swerve.getImu()::zeroAll),
        //     Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
        //     Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            // Commands.sequence(
            //     intake.setEnabledCommand(true),
            //     Commands.race(
            //         Commands.parallel(
            //             AutoBuilder.followPath(pathGroup.get(0)),
            //             intake.intakeCommand()
            //         ),
            //         Commands.waitSeconds(5)
            //     ),
            //     intake.setEnabledCommand(false),
            //     Commands.race(
            //         AutoBuilder.followPath(pathGroup.get(1)),
            //         Commands.waitSeconds(3)
            //     ),
            //     intake.setEnabledCommand(true),
            //     intake.outtakeCommand(),
            //     Commands.waitSeconds(3),
            //     intake.setEnabledCommand(false)
            // )
        // );  
    }
}

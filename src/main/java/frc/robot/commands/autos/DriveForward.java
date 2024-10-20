// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class DriveForward extends SequentialCommandGroup {
  private final SwerveDrivetrain swerveDrive;
  private final IntakeRoller intakeRoller;

  public DriveForward(SwerveDrivetrain swerve, IntakeRoller intake) {
    this.swerveDrive = swerve;
    this.intakeRoller = intake;

    addCommands(
      Commands.run(()->swerveDrive.drive(2,0,0))
      // Commands.sequence(
      //   intakeRoller.setEnabledCommand(true),
      //   Commands.race(
      //     Commands.parallel(
      //       Commands.run(() -> swerveDrive.drive(1.0, 0, 0)),  
      //       intakeRoller.intakeCommand()
      //     ),
      //     Commands.waitSeconds(5)
      //   ),
      //   intakeRoller.setEnabledCommand(false),
      //   Commands.runOnce(() -> swerveDrive.drive(0,0,0)),
      //   Commands.race(
      //     Commands.run(() -> swerveDrive.drive(-1.0, 0, 0)),
      //     Commands.waitSeconds(3)
      //   ),
      //   Commands.runOnce(() -> swerveDrive.drive(0,0,0)),
      //   intakeRoller.setEnabledCommand(true),
      //   intakeRoller.outtakeCommand(),
      //   Commands.waitSeconds(3),
      //   intakeRoller.setEnabledCommand(false)
      // )
    );
      
    
  }
}

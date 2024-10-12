package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimbConstants.ClimbPostions;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TrapConstants;
import frc.robot.Constants.BeambreakConstants;
// import frc.robot.subsystems.vision.DriverAssist;
// import frc.robot.subsystems.vision.ShooterVisionAdjustment;
import frc.robot.util.NerdyLine;
import frc.robot.util.NerdyMath;

import frc.robot.subsystems.swerve.SwerveDrivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SuperSystem {
    public IntakeRoller intakeRoller;
    public Indexer indexer;
    public ShooterPivot shooterPivot;
    public ShooterRoller shooterRoller;
    public Tramp tramp;
    public BeamBreakSensor intakeBeamBreak;
    public BeamBreakSensor trampBeamBreak;
    public BeamBreakSensor shooterBeamBreak;
    public Climb climb;

    private double[] distances = {0,0,0,0,0,0}; // distances are old
    private double[] angles = {0,0,0,0,0,0}; // angles are old
    private double lastAngle = -0.2;
    private double angleOffset = 0.0;
    private NerdyLine angleLine;

    public SuperSystem(IntakeRoller intakeRoller, Indexer indexer, ShooterPivot shooterPivot, ShooterRoller shooterRoller, Tramp tramp, Climb climb, BeamBreakSensor intakeBeamBreak, BeamBreakSensor trampBeamBreak, BeamBreakSensor shooterBeamBreak) {
        this.intakeRoller = intakeRoller;
        this.indexer = indexer;
        this.shooterPivot = shooterPivot;
        this.shooterRoller = shooterRoller;
        this.tramp = tramp;
        this.intakeBeamBreak = intakeBeamBreak;
        this.trampBeamBreak = trampBeamBreak;
        this.shooterBeamBreak = shooterBeamBreak;
        this.climb = climb;
    }

    public Command getReadyForAmp() {
        Command command = Commands.sequence(
            Commands.runOnce(() -> tramp.elevatorAmp()),
            tramp.setEnabledCommand(true)
            
        ).finallyDo(
            () -> {
                tramp.stop();
            }
        );
        return command;
    }

    // public Command shootAmp() {
    //     Command command = Commands.sequence(
    //         tramp.settrampShootCommand(),
    //         tramp.setRollerEnabledCommand(false)
           
    //     ).finallyDo(
    //         () -> {
    //            tramp.elevatorDown();
    //         }

    //     );
    //     return command;
    //     }

    public Command stow() {
        Command command = Commands.sequence(
            intakeRoller.stopCommand(),
            //shooterRoller.stopCommand(),
            indexer.stopCommand()
            //shooterPivot.setPositionCommand(ShooterConstants.kFullStowPosition.get())
        );

        command.addRequirements(indexer, intakeRoller); // Removed shooterPivot, shooterRoller

        return command;
    }

    public Command backupIndexer() {
        Command command = Commands.sequence(
            indexer.setEnabledCommand(true),
            indexer.reverseIndexCommand(),
            Commands.waitSeconds(0.2),
            indexer.stopCommand()
        ).finallyDo(indexer::stop);

        command.addRequirements(indexer);
        
        return command;
    }

    public Command backupIndexerAndShooter() {
        Command command = Commands.sequence(
            shooterRoller.setVelocityCommand(-20, -20),
            shooterRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            indexer.reverseIndexCommand(),
            Commands.waitSeconds(0.2),
            indexer.stopCommand(),
            shooterRoller.stopCommand()
        ).finallyDo(() -> {
            indexer.stop();
            shooterRoller.stop();    
        });

        command.addRequirements(indexer, shooterRoller);
        
        return command;
    }

    public Command backupIndexerManual() {
        Command command = Commands.sequence(
            // shooterPivot.moveToSpeaker(), Ignore Comment
            indexer.setEnabledCommand(true),
           // shooterRoller.setEnabledCommand(true),
            indexer.reverseIndexCommand(),
           // shooterRoller.setReverseVelocityCommand(-10, -10), // TODO: Later
            Commands.waitSeconds(0.2)
        ).finallyDo(() -> {
            indexer.stop();
           // shooterRoller.stop();
        });

        command.addRequirements(indexer); // Removed shooterRoller
        
        return command.withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command intakeUntilSensed(){
        Command command = Commands.sequence(
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            shooterRoller.setEnabledCommand(true),
            intakeRoller.intakeCommand(),
            indexer.indexToShooterCommand(),
            shooterRoller.setVelocityCommand(-1),
            Commands.waitUntil(shooterBeamBreak::noteSensed)

        ).finallyDo(() -> {
            intakeRoller.stop();
            indexer.stop();
            shooterRoller.stop();
        });

        command.addRequirements(indexer, intakeRoller);
        return command;
    }

    public Command shooterToTramp() {
        Command command = Commands.sequence(
            tramp.setEnabledCommand(true),
            tramp.setElevatorDownCommand(),
            Commands.deadline(
                Commands.waitUntil(() -> tramp.hasReachedPosition(TrapConstants.kElevatorDownPosition)),
                Commands.waitSeconds(1)
            ),
            shooterRoller.setEnabledCommand(true),
            shooterRoller.setLeftVelocityCommand(1), // Change Later
            indexer.setEnabledCommand(true),
            indexer.setVelocityCommand(-1),
            Commands.waitUntil(trampBeamBreak::noteSensed)
        ).finallyDo(() -> {
            indexer.stop();
            shooterRoller.stop();
        });
        command.addRequirements(indexer, shooterPivot, shooterRoller);
        return command;
    }

    public Command shoot() { // copypasted from old code
        Command command = Commands.either(
            // Pass
            Commands.sequence(
                Commands.runOnce(() -> isPassing = true),
                shooterPivot.setPositionCommand(ShooterConstants.kSpeakerPosition.get()),
                shooterRoller.setEnabledCommand(true),
                shooterRoller.setVelocityCommand(37),
                Commands.waitSeconds(0.3),
                indexer.setEnabledCommand(true),
                indexer.indexCommand()
            ),
            // Shoot
            Commands.sequence(
                Commands.runOnce(() -> isPassing = false),
                indexer.setEnabledCommand(true),
                indexer.indexCommand()
            ),
            () -> shooterRoller.getTargetVelocityLeft() == 0
               && shooterRoller.getTargetVelocityRight() == 0
        );

        command.addRequirements(indexer);

        return command;
    }

    public Command shootSpeaker() {
        Command command = Commands.sequence(
            shooterRoller.setEnabledCommand(true),
            shooterRoller.shootSpeakerRight(),
            Commands.deadline(
                Commands.waitUntil(() ->
                    shooterRoller.atTargetVelocityRight()),
                Commands.waitSeconds(1)
            ),
            shooterPivot.setEnabledCommand(true),
            shooterPivot.moveToSpeaker(),
            shooterRoller.setLeftVelocityCommand(-1)
        ).finallyDo(() -> {
            indexer.stop();
            shooterPivot.stop();
            shooterRoller.stop();
        });

        command.addRequirements(indexer, shooterPivot, shooterRoller);
        return command;
    }

    public Command shootAmp() {
        Command command = Commands.sequence(
            tramp.setEnabledCommand(true),
            tramp.setElevatorAmpCommand(),
            tramp.setTrampShootCommand()
        ).finallyDo(() -> {
            tramp.setElevatorDownCommand();
            tramp.stop();
        });
        return command;
    }

    public Command shootSpeakerAutoAim() {
        Command command = Commands.sequence(
            shooterRoller.shootSpeaker(),
            shooterRoller.setEnabledCommand(true),

            Commands.deadline(
                Commands.waitUntil(() -> 
                    shooterRoller.atTargetVelocityLeft() &&
                    shooterRoller.atTargetVelocityRight()),
                Commands.waitSeconds(1)
            ),
            shooterPivot.moveToSpeaker(),
            shooterPivot.setEnabledCommand(true),
            indexer.indexToShooterCommand(),
            indexer.setEnabledCommand(true)
        ).finallyDo(() -> {
            indexer.stop();
            shooterPivot.stop();
            shooterRoller.stop();
        });

        command.addRequirements(indexer, shooterPivot, shooterRoller);
        return command;
    }

    public Command intakeBasic() {
        Command command = Commands.sequence(
            shooterPivot.setEnabledCommand(true),
            shooterPivot.moveToNeutral(),
            Commands.deadline(  // check if correct pls
                Commands.waitUntil(() -> 
                    shooterPivot.hasReachedPosition(ShooterConstants.kNeutralPosition.get())),
                // handoff(),
                Commands.waitSeconds(1)
            ),
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Intaking", true)),
            indexer.indexCommand(),
            intakeRoller.intakeCommand(),
            Commands.waitUntil(() -> false)
        ).finallyDo(
            () -> {
                SmartDashboard.putBoolean("Intaking", false);
                intakeRoller.stop();
                indexer.stop();
            }
        );

        command.addRequirements(indexer, intakeRoller); // Removed Shooter Pivot and Shooter Roller
        return command;
    }

    public Command intakeBasicHold() {
        Command command = Commands.sequence(
            Commands.deadline(
                /*Commands.waitUntil(() -> 
                    shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
                handoff(),
                Commands.waitSeconds(1)
                ),
                */
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Intaking", true))
            ),
            indexer.indexCommand(),
            intakeRoller.intakeCommand()
        );

        command.addRequirements(indexer, intakeRoller); // Removed shooterPivot, shooterRoller
        return command;
    }

    public Command stopIntaking() {
        Command command = Commands.sequence(
        indexer.reverseIndexCommand(),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> {
                SmartDashboard.putBoolean("Intaking", false);
                intakeRoller.stop();
                indexer.stop();
            })
        );

        command.addRequirements(indexer, intakeRoller);
        return command;
    }

    public Command eject() {
        Command command = Commands.sequence(
            /*Commands.deadline(
                Commands.waitUntil(() -> 
                shooterPivot.hasReachedPosition(ShooterConstants.kEjectPosition.get())),
                shooterPivot.setPositionCommand(ShooterConstants.kEjectPosition.get()),
                Commands.waitSeconds(0.5)
            ),
            */
            intakeRoller.setEnabledCommand(true),
            intakeRoller.setVelocityCommand(-100),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Outtaking", true)),
            Commands.waitSeconds(0.25),
            indexer.setEnabledCommand(true),
            indexer.setVelocityCommand(-50),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Intake roller", true)),
            Commands.waitUntil(() -> false)
        ).finallyDo(
            () -> {
                SmartDashboard.putBoolean("Intake roller", false);
                SmartDashboard.putBoolean("Outtaking", false);
                intakeRoller.stop();
                indexer.stop();
            }
        );

        command.addRequirements(indexer, intakeRoller); // Removed shooterPivot, shooterRoller
        return command;
    }

    public Command panicEject() {
        Command command = Commands.sequence(
            //shooterPivot.setPositionCommand(ShooterConstants.kFullStowPosition.get() + 2),
            intakeRoller.setEnabledCommand(true),
            intakeRoller.setVelocityCommand(-50),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Outtaking", true)),
            indexer.setEnabledCommand(true),
            indexer.setVelocityCommand(-50),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Intake roller", true)),
            Commands.waitUntil(() -> false)
        ).finallyDo(
            () -> {
                SmartDashboard.putBoolean("Intake roller", false);
                SmartDashboard.putBoolean("Outtaking", false);
                intakeRoller.stop();
                indexer.stop();
            }
        );

        command.addRequirements(indexer, intakeRoller); // Removed shooterPivot, shooterRoller
        return command;
    }

    public Command ejectIntakeOnly() {
        Command command = Commands.sequence(
            intakeRoller.setEnabledCommand(true),
            intakeRoller.setVelocityCommand(-100)
        ).finallyDo(
            () -> {
                intakeRoller.stop();
            }
        );

        command.addRequirements(intakeRoller);
        return command;
    }

    public Command climbSequenceHoldCommand(){
        Command command = Commands.sequence(
            climb.setEnabledCommand(true),
            climb.setPositionStateTopCommand()
        );
        return command;
        
    }
    
    public Command climbSequenceRealeaseCommand(){
        Command command = Commands.sequence(
            climb.setEnabledCommand(true),
            climb.setPositionStateBottomCommand()
        );
        return command; 
    }

    public double getShooterAngle(SwerveDrivetrain swerve)
    {
        return getShooterAngle(swerve, false);
    }
    
    public void incrementOffset(double increment)
    {
        angleOffset += increment;
        angleOffset = NerdyMath.clamp(angleOffset, -10, 10);
    }
    
    public void resetOffset()
    {
        angleOffset = 0;
    }

    public double getShooterAngle(SwerveDrivetrain swerve, boolean preserveOldValue)
    {
        double distance = swerve.getDistanceFromTag(preserveOldValue, RobotContainer.IsRedSide() ? 4 : 7);
        if(distance < distances[0]) {
            SmartDashboard.putBoolean("Vision failed", true);
            return (preserveOldValue ? lastAngle : -0.2);
        }
        if (distance > distances[distances.length - 1]) {
            SmartDashboard.putBoolean("Vision failed", true);
            return (preserveOldValue ? lastAngle : -0.3);
        }
        
        double output = NerdyMath.clamp(angleLine.getOutput(distance), ShooterConstants.kFullStowPosition.get(), 20);
        output += angleOffset;
        lastAngle = output;
        return output + 1.5;
    }

    private boolean isPassing = false;

    public boolean getIsPassing() {
        return isPassing;
    }

}
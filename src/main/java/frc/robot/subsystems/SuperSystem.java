package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TrapConstants;
//import frc.robot.subsystems.vision.DriverAssist;
//import frc.robot.subsystems.vision.ShooterVisionAdjustment;
import frc.robot.util.NerdyLine;
import frc.robot.util.NerdyMath;

import frc.robot.subsystems.swerve.SwerveDrivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SuperSystem {
    public IntakeRoller intakeRoller;
    public IndexerV2 indexer;
    public ShooterPivot shooterPivot;
    public ShooterRoller shooterRoller;
    public Tramp tramp;
    public BannerSensor bannerSensor;

    private double[] distances = {1.2,   2.483,   3.015,    3.573,   4.267,   4.697}; // distances from 4/6
    private double[] angles = {-52.470, -32.861, -29.114, -25.663, -21.413, -20.8}; // angles from 4/6
    private double lastAngle = -0.2;
    private double angleOffset = 0.0;
    private NerdyLine angleLine;

    public SuperSystem(IntakeRoller intakeRoller, IndexerV2 indexer, ShooterPivot shooterPivot, ShooterRoller shooterRoller, Tramp tramp) {
        this.intakeRoller = intakeRoller;
        this.indexer = indexer;
        this.shooterPivot = shooterPivot;
        this.shooterRoller = shooterRoller;
        this.tramp = tramp;
        this.bannerSensor = new BannerSensor();
    }

    public boolean noteIntook() {
        // return colorSensor.noteIntook() || bannerSensor.noteIntook();
        return bannerSensor.noteIntook();
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

     public Command panicButton() {
        Command command = Commands.sequence(
           // shooterPivot.setPositionCommand(6),
           // shooterRoller.setVelocityCommand(-20),
           // shooterRoller.setEnabledCommand(true),
            indexer.indexCommand(),
            indexer.setEnabledCommand(true),
            Commands.waitUntil(() -> false)
        ).finallyDo(
            () -> {
              //  shooterRoller.stop();
                indexer.stop();
            }
        );

        command.addRequirements(indexer); // Deleted Shooter Pivot and Shooter Roller

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

    public Command intakeUntilSensed() {
        Command command = Commands.sequence(
            // Commands.deadline(
            //     /*Commands.waitUntil(() -> 
            //         shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
            //     handoff(),
            //     Commands.waitSeconds(1)
            //     */
            // ),
           // shooterRoller.setVelocityCommand(0, 0),
           // shooterRoller.setEnabledCommand(true),
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            indexer.indexCommand(),
            intakeRoller.intakeCommand(),

            // Commands.deadline(
                // Commands.waitSeconds(1), // testing - check wait time             
            Commands.waitUntil(this::noteIntook),
            // ),
            
            // Move note back
            intakeRoller.stopCommand(),
            indexer.reverseIndexCommand(),
           // shooterRoller.setVelocityCommand(0, 0),
            Commands.waitSeconds(0.2), // Was 0.6   3/3/24   Code Orange

            indexer.stopCommand()
            //shooterRoller.stopCommand()
        ).finallyDo(() -> {
            intakeRoller.stop();
            indexer.stop();
           // shooterRoller.stop();
        });

        command.addRequirements( indexer, intakeRoller); // Removed Shooterintake and ShooterRoller
        return command;
    }

    public Command intakeUntilSensedNoBackup() {
        Command command = Commands.sequence(
            // Commands.deadline(
            //     /*Commands.waitUntil(() -> 
            //         shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
            //     handoff(),
            //     Commands.waitSeconds(1)*/
            // ),
           // shooterRoller.setVelocityCommand(-10, -10),
            //shooterRoller.setEnabledCommand(true),
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            indexer.indexCommand(),
            intakeRoller.intakeCommand(),

            Commands.waitUntil(this::noteIntook),
            intakeRoller.stopCommand(),
            indexer.stopCommand(),
            shooterRoller.stopCommand()
        ).finallyDo(() -> {
            intakeRoller.stop();
            indexer.stop();
            //shooterRoller.stop();
        });

        command.addRequirements( indexer, intakeRoller); //Removed Shooter Pivot and Shooter Roller
        return command;
    }

    public Command intakeUntilSensedAuto(double timeout) {
        Command command = Commands.sequence(
            // Commands.deadline(
            //     /*Commands.waitUntil(() -> 
            //         shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
            //     handoff(),
            //     Commands.waitSeconds(1)*/
            // ),
            //shooterRoller.setVelocityCommand(-10, -10),
           // shooterRoller.setEnabledCommand(true),
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            indexer.indexCommand(),
            intakeRoller.intakeCommand(),

            Commands.deadline(
                Commands.waitSeconds(timeout), // testing - check wait time             
                Commands.waitUntil(this::noteIntook)
            )
        ).finallyDo(() -> {
            intakeRoller.stop();
            indexer.stop();
            //shooterRoller.stop();
        });

        command.addRequirements(indexer, intakeRoller); // Removed ShooterPivot and ShooterRoller
        return command;
    }

    public Command intakeNew() {
        Command command = Commands.sequence(
            shooterPivot.setEnabledCommand(true),
            Commands.deadline(
                Commands.waitUntil(() -> 
                    shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
                Commands.waitSeconds(1)
            ),
            shooterRoller.setEnabledCommand(true),
            shooterRoller.setVelocityCommand(0, 0),
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            indexer.indexToShooterCommand(),
            intakeRoller.intakeCommand()
        ).finallyDo(() -> {
            intakeRoller.stop();
            indexer.stop();
            shooterRoller.stop();
        });

        command.addRequirements(indexer, intakeRoller, shooterPivot, shooterRoller);
        return command;
    }

    public Command intakeToElevator() {
        Command command = Commands.sequence(
            tramp.setElevatorDownCommand(),
            tramp.setEnabledCommand(true),
            Commands.deadline(
                Commands.waitUntil(() -> 
                tramp.hasReachedPosition(TrapConstants.kElevatorDownPosition)),
                Commands.waitSeconds(1)
            ),
            shooterRoller.setVelocityCommand(0, 0),
            shooterRoller.setEnabledCommand(true),
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            indexer.indexToElevatorCommand(),
            intakeRoller.intakeCommand()
        ).finallyDo(() -> {
            intakeRoller.stop();
            indexer.stop();
            shooterRoller.stop();
        });

        command.addRequirements(indexer, intakeRoller, shooterPivot, shooterRoller);
        return command;
    }

    public Command shootSpeaker() {
        Command command = Commands.sequence(
            shooterRoller.shootSpeakerRight(),
            shooterRoller.setEnabledCommand(true),
            Commands.deadline(
                Commands.waitUntil(() ->
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
            indexer.setVelocityCommand(-50, 50),
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

    private boolean isPassing = false;

    public boolean getIsPassing() {
        return isPassing;
    }

}
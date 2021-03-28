package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveCommand extends CommandBase {
  /**
   * Creates a new DriveCommand.
   */

  DriveTrain driveTrain;
  DoubleSupplier throttle;
  DoubleSupplier turn;
  BooleanSupplier driveSwitch;

  public DriveCommand(DriveTrain driveTrain, DoubleSupplier throttle, DoubleSupplier turn, BooleanSupplier driveSwitch) {
    this.driveTrain = driveTrain;
    this.throttle = throttle;
    this.turn = turn;
    this.driveSwitch = driveSwitch;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Use squared inputs
    // Might want to change that for competition
    driveTrain.allDrive(throttle.getAsDouble(), turn.getAsDouble(), true, driveSwitch.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
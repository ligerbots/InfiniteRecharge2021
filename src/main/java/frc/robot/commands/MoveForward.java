// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


public class MoveForward extends CommandBase {
  private DriveTrain driveTrain;
  private DriveCommand driveCommand;
  private Pose2d start;

  // distance that must be travelled in meters
  private double distance = 2.0;

  /** Creates a new MoveForward. 
 * @param carouselCommand*/
  public MoveForward(DriveTrain driveTrain, DriveCommand driveCommand) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // takes note of where the robot started and then drives forward
    start = driveTrain.getPose();
    driveTrain.tankDriveVolts(12.0, 12.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // nothing to do while the robot is moving
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stops the robot
    driveTrain.tankDriveVolts(0.0, 0.0);
    driveCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // will return true if the robot has traveled 2 meters. This method uses the pythagorean theorem to find the distance
    return Math.sqrt(
            Math.pow(Math.abs(driveTrain.getPose().getX() - start.getX()), 2) + 
            Math.pow(Math.abs(driveTrain.getPose().getY() - start.getY()), 2))  
          >= distance;
  } 
}


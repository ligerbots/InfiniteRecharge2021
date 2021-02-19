/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToHeading extends CommandBase {
  /**
   * Creates a new FaceShootingTarget.
   */
  double acceptableError;

  double targetHeading;
  double deltaAngle;

  DriveTrain robotDrive;
  DriveCommand driveCommand;

  public TurnToHeading(DriveTrain robotDrive, double acceptableError, DriveCommand driveCommand, double targetHeading) {
    this.robotDrive = robotDrive;
    this.acceptableError = acceptableError;
    this.driveCommand = driveCommand;
    this.targetHeading = targetHeading;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveCommand.cancel();   
    deltaAngle = Math.PI;
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
      double currentHeading = robotDrive.getHeading();
      deltaAngle = currentHeading - targetHeading;
      if(deltaAngle > 180)  deltaAngle-= 360;
      if(deltaAngle < -180)  deltaAngle+= 360;
      double turnspeed=robotDrive.turnSpeedCalc(deltaAngle);

      // System.out.format("FaceShootingTarget: %3.2f%n", initialAngleOffset);
      // curvature drive doesn't work so use arcade drive
      robotDrive.arcadeDrive(0, turnspeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotDrive.allDrive(0, 0, false);
    driveCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(deltaAngle) < acceptableError;
  }
}

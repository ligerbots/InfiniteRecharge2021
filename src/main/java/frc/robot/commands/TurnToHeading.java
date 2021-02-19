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
  DriveTrain driveTrain;

  double acceptableError;
  double targetHeading;
  double deltaAngle;

  public TurnToHeading(DriveTrain driveTrain, double targetHeading, double acceptableError) {
    this.driveTrain = driveTrain;
    this.acceptableError = acceptableError;
    this.targetHeading = targetHeading;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set deltaAngle high to make sure it does at least one loop 
    deltaAngle = Math.PI;
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
      double currentHeading = driveTrain.getHeading();
      deltaAngle = currentHeading - targetHeading;
      if (deltaAngle > 180.0)  deltaAngle -= 360.0;
      if (deltaAngle < -180.0)  deltaAngle += 360.0;
      double turnspeed = driveTrain.turnSpeedCalc(deltaAngle);

      // curvature drive cannot turn without moving, so make sure to use arcade drive
      driveTrain.arcadeDrive(0, turnspeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.allDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(deltaAngle) < acceptableError;
  }
}

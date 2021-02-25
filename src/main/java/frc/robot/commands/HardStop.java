// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class HardStop extends CommandBase {
  /** Creates a new HardStop. */

  DriveTrain driveTrain;

  private final double driveTime = 0.055; 
  private double startTime;

  public HardStop(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // sets the motors to the opposte direction and half speed 
    driveTrain.tankDriveVolts(-2. * Math.signum(driveTrain.getLeftSpeed()), -2. * Math.signum(driveTrain.getRightSpeed())); 
    startTime = Robot.time();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //nothing to do while the robot waits for the timer
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stops robot
    driveTrain.tankDriveVolts(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.time() - startTime >= driveTime;
  }
}

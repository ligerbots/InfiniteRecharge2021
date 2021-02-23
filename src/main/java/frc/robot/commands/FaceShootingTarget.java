/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision.VisionMode;

public class FaceShootingTarget extends CommandBase {
  /**
   * Creates a new FaceShootingTarget.
   */
  double startingAngle;
  double acceptableError;
  DriveTrain robotDrive;
  DriveCommand driveCommand;
  Shooter shooter;

  boolean oldOldCheck;
  boolean oldCheck;
  boolean check;

  boolean targetAcquired;
  double headingError;
  double headingTarget;

  double startTime;

  public FaceShootingTarget(DriveTrain robotDrive, double acceptableError, DriveCommand driveCommand, Shooter shooter) {
    this.robotDrive = robotDrive;
    this.acceptableError = acceptableError;
    this.driveCommand = driveCommand;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setTurretAdjusted(Constants.TURRET_ANGLE_ZERO_SETTING);
    targetAcquired = false;
    oldOldCheck = false;
    check = false;
    oldCheck = false;
    if (driveCommand != null) driveCommand.cancel();
    shooter.vision.setMode(VisionMode.GOALFINDER);
    startingAngle = robotDrive.getHeading();
    startTime = Robot.time();
    headingError = 100000;
    System.out.println("Initial initial heading: " + startingAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (targetAcquired) {
      headingError = robotDrive.getHeading() - headingTarget;
      while ( headingError > 180.0) headingError -= 360.0;
      while ( headingError < -180.0) headingError += 360.0;
      SmartDashboard.putNumber("shooter/HeadingError", headingError);
      
      check = Math.abs(headingError) < acceptableError && oldCheck;
      // System.out.format("FaceShootingTarget: %3.2f%n", initialAngleOffset);
      robotDrive.arcadeDrive(0, robotDrive.turnSpeedCalc(headingError));

      oldCheck = Math.abs(headingError) < acceptableError && oldOldCheck;

      oldOldCheck = Math.abs(headingError) < acceptableError;
    }
    else if (shooter.vision.getStatus())
    {
      headingTarget = startingAngle - shooter.vision.getRobotAngle();
      System.out.println("STARTING ANGLE "+ startingAngle);
      System.out.println("GET ROBOT ANGLE "+ shooter.vision.getRobotAngle());

      System.out.println("HEADING TARGET "+headingTarget);
      targetAcquired = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("FACE SHOOTING FINISHED");
    System.out.println("Current Heading: " + robotDrive.getHeading() + System.getProperty("line.separator") + 
        "Target Angle: " + headingTarget);

    robotDrive.allDrive(0, 0, false);
    if (driveCommand != null) driveCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(headingError) < acceptableError && check) 
        || (!targetAcquired && (Robot.time() - startTime) > 0.5);
  }
}

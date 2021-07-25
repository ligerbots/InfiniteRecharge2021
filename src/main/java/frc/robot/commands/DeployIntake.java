// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;

public class DeployIntake extends CommandBase {
  Climber climber;
  boolean started;
  double startTime;

  public DeployIntake(Climber climber){
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // tests to see if we have started deploying the robot yet ;P
    started = false;
    climber.shoulder.setIdleMode(IdleMode.kBrake);
    startTime = Robot.time();
  }

  @Override
  public void execute() {
    // raise the shoulder a little so that the intake can unhook at the start of the match
    if (!started) {    // .589 = angle with leg
      climber.shoulder.setVoltage(Constants.SHOULDER_SPEED_UP);
      // set started to true so that we don't raise the shoulder again.
      if (climber.shoulderEncoder.get() > Constants.SHOULDER_RELEASE_HEIGHT) started = true;
    }

    // slowly lower the shoulder
    else {
      if ( !climber.shoulderBelowHeight(15.0) ) {
        climber.shoulder.setVoltage(Constants.SHOULDER_SPEED_DOWN_FAST);
      }
      else {
        climber.shoulder.setVoltage(Constants.SHOULDER_SPEED_DOWN_SLOW);
      }
    }
  }
    
  @Override
  public void end(boolean interrupted) {
      climber.shoulder.setVoltage(0.0);
      climber.shoulder.setIdleMode(IdleMode.kCoast);
      System.out.println("DeployIntake ended. interrupted = " + interrupted);
      climber.switchDeployed();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getShoulderPosition() <= Constants.SHOULDER_MIN_VELOCITY_HEIGHT
        || Robot.time() - startTime > 2.0;
  }
}

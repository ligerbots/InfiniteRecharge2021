// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class DeployIntake extends CommandBase {
  Climber climber;
  boolean started;
  final static double unhookIntakeSpeed = 0.5;
  final static double lowerIntakeSpeed = -0.1;

  public DeployIntake(Climber climber){
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //tests to see if we have started deploying the robot yet ;P
    started = false;
    climber.shoulder.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void execute() {
    //raise the shoulder a little so that the intake can unhook at the start of the match
    if(!started){
      climber.shoulder.setVoltage(unhookIntakeSpeed);
      //set started to true so that we don't raise the shoulder again.
      started = true;
    }

    //slowly lower the shoulder
    else{
      if(!climber.shoulderAtMinHeight()) {
      climber.shoulder.setVoltage(lowerIntakeSpeed);
      }
    }
  }
    
  @Override
  public void end(boolean interrupted) {
      climber.shoulder.setVoltage(0.0);
      climber.shoulder.setIdleMode(IdleMode.kCoast);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getShoulderPosition() <= Constants.SHOULDER_MIN_VELOCITY_HEIGHT;
  }
}

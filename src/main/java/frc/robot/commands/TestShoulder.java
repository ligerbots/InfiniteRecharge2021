// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class TestShoulder extends CommandBase {
    Climber climber;
    public TestShoulder(Climber climber){
      this.climber = climber;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.shoulder.setVoltage(Constants.SHOULDER_SPEED_HOLD);
    climber.shoulder.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void execute() {
    if(!climber.shoulderBelowMinHeight()) {
      climber.shoulder.setVoltage(-0.1);
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

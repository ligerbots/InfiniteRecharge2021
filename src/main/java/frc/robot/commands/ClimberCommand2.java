/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberCommand2 extends CommandBase {
  /**
   * Creates a new ClimberCommand2.
   */
  Climber climber;

  enum ClimbingPhase {
    LOWER_WINCH, AUTO_LEVEL, FINISHED
  }

  ClimbingPhase currentPhase;

  public ClimberCommand2(Climber climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPhase = ClimbingPhase.LOWER_WINCH;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(currentPhase + "    " + climber.getWinchPosition());
    
    switch (currentPhase) {
      case LOWER_WINCH:
        climber.moveWinch(Constants.WINCH_CLIMB_HEIGHT - 200.0);
        if (climber.getWinchPosition() < Constants.WINCH_CLIMB_HEIGHT - 190.0) {
          currentPhase = ClimbingPhase.AUTO_LEVEL;
        }
        // Note: falls through

      case AUTO_LEVEL:
        climber.autoLevel(true);
        currentPhase = ClimbingPhase.FINISHED;
        break;

      case FINISHED:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentPhase == ClimbingPhase.FINISHED;
  }
}

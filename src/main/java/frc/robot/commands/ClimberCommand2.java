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

  // enum ClimbingPhase {
  //   LOWER_WINCH, AUTO_LEVEL, FINISHED
  // }

  // ClimbingPhase currentPhase;

  public ClimberCommand2(Climber climber) {
    this.climber = climber;
  }

  // NOTE: Paul Rensing July 2021
  //  We don't actually lower the Shoulder in the 2nd phase, because we only raised the shoulder once.
  //  See simplified command below. This is left for documentation purposes.
  
  // // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {
  //   currentPhase = ClimbingPhase.LOWER_WINCH;
  // }

  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  //   System.out.println(currentPhase + "    " + climber.getWinchPosition());
    
  //   // NOTE: (Paul Rensing, July 28, 2021)  This is not really a "state machine".
  //   //  Because the switch for LOWER_WINCH falls through, we *always* turn on autoLevel.
  //   //  The only thing is we stop checking the winch once it is done.
  //   switch (currentPhase) {
  //     case LOWER_WINCH:
  //       // Move shoulder down so hook goes down straighter as we lower the winch.
  //       // Not needed when no RAISE_SHOULDER2 ?
  //       // climber.moveShoulder(Constants.SHOULDER_HEIGHT_FOR_RAISE1);

  //       climber.moveWinch(Constants.WINCH_CLIMB_HEIGHT);

  //       This test is incorrect; winch ticks always increases.
  //       if (climber.getWinchPosition() < Constants.WINCH_CLIMB_HEIGHT - 10.0) {
  //         currentPhase = ClimbingPhase.AUTO_LEVEL;
  //       }
  //       // Note: falls through

  //     case AUTO_LEVEL:
  //       climber.autoLevel(true);
  //       currentPhase = ClimbingPhase.FINISHED;
  //       break;

  //     case FINISHED:
  //       break;
  //   }
  // }

  @Override
  public void execute() {
    // Set the desired winch ticks position and turn on autolevel.
    // The code in Climber.periodic does the rest.
    climber.moveWinch(Constants.WINCH_CLIMB_HEIGHT);
    climber.autoLevel(true);
  }

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {
  // }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
    // return currentPhase == ClimbingPhase.FINISHED;
  }
}

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
import frc.robot.subsystems.Climber;

public class ClimberCommand1 extends CommandBase {
  /**
   * Creates a new ClimberCommand.
   */
  Climber climber;

  enum ClimbingPhase {
    RAISE_SHOULDER1, RAISE_WINCH, RAISE_SHOULDER2, FINISHED
  }

  ClimbingPhase currentPhase;

  public ClimberCommand1(Climber climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    SmartDashboard.putString("ClimberCmd1 Phase", "none");
    SmartDashboard.putNumber("ClimberCmd1 Winch", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPhase = ClimbingPhase.RAISE_SHOULDER1;
    climber.switchDeployed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("ClimberCmd1 Winch", climber.getWinchPosition());

    switch (currentPhase) {
      case RAISE_SHOULDER1:
        climber.moveShoulder(Constants.SHOULDER_HEIGHT_FOR_RAISE1);
        if (climber.shoulderOnTarget()) {
          currentPhase = ClimbingPhase.RAISE_WINCH;
        }
        System.out.println("Shoulder Position: " + climber.getShoulderPosition());
        break;

      case RAISE_WINCH:
        climber.moveWinch(Constants.WINCH_MAX_HEIGHT_TICK_COUNT);
        if (climber.getWinchPosition() >= Constants.WINCH_MAX_HEIGHT_TICK_COUNT - 10.0) {
          // currentPhase = ClimbingPhase.RAISE_SHOULDER2;
          // From March 2020, do not actually raise the shoulder
          currentPhase = ClimbingPhase.FINISHED;
        }
        System.out.println(" " + climber.getWinchPosition());
        break;

      case RAISE_SHOULDER2:
        climber.moveShoulder(Constants.SHOULDER_HEIGHT_FOR_MAX_CLIMB);
        currentPhase = ClimbingPhase.FINISHED;
        break;

      case FINISHED: 
        break;
    }

    SmartDashboard.putString("ClimberCmd1 Phase", currentPhase.toString());
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

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ManualLowerWinchCommand extends CommandBase {
    Climber climber;
    double initialWinchPosition;

    public ManualLowerWinchCommand(Climber climber){
        this.climber = climber;
    }

    public void initialize() {
        initialWinchPosition = climber.getWinchPosition();
    }
    
    @Override
    public void execute() {
        climber.moveWinch(initialWinchPosition - 15);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
    return (Math.abs(climber.getWinchPosition() + 15 - initialWinchPosition ) < 1 ); //check for margin of error
    } 
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ManualLowerWinchCommand extends CommandBase {
    Climber climber;

    public ManualLowerWinchCommand(Climber climber){
        this.climber = climber;
    }

    public void initialize() {
        // note: the winch position always increases
        climber.moveWinch(climber.getWinchPosition() + 50);
        // climber.moveShoulder(climber.getShoulderPosition() - 0.05);
    }
    
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    } 
}

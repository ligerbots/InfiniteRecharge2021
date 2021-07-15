package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;

public class MoveForwardAuto extends SequentialCommandGroup{

    public MoveForwardAuto(DriveTrain driveTrain, DriveCommand driveCommand, Climber climber){
        DeployIntake deployIntake = new DeployIntake(climber);
    
        addCommands(deployIntake, new MoveForward(driveTrain, driveCommand));
    }
}

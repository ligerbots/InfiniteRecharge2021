package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldMap;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;

public class MoveForwardAuto extends SequentialCommandGroup implements AutoCommandInterface {

    public MoveForwardAuto(DriveTrain driveTrain, DriveCommand driveCommand/*, Climber climber*/) {
        // DeployIntake deployIntake = new DeployIntake(climber);
    
        addCommands(new MoveForward(driveTrain, driveCommand));
    }

    public Pose2d getInitialPose() {
        return FieldMap.startPosition[0];
    }
}

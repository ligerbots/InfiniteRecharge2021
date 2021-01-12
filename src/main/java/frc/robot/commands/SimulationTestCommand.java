package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SimulationTestCommand extends CommandBase implements AutoCommandInterface {
    DriveTrain driveTrain;

    public SimulationTestCommand(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }

    @Override
    public void execute() {
        driveTrain.allDrive(Math.sin(System.currentTimeMillis()*2e-3), 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(5.,5.,new Rotation2d(0.));
    }
}

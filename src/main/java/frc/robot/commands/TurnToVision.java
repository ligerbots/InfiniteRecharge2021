package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class TurnToVision extends CommandBase implements AutoCommandInterface {
    Vision vision;
    DriveTrain driveTrain;
    Rotation2d targetDirection;
    Rotation2d currentDirection;
    Rotation2d deltaDirection;

    public TurnToVision(Vision vision, DriveTrain driveTrain, Rotation2d targetDirection) {
        this.vision = vision;
        this.driveTrain = driveTrain;
        this.targetDirection = targetDirection;
    }

    @Override
    public void initialize() {
        currentDirection = Rotation2d.fromDegrees(driveTrain.getHeading());
        deltaDirection = this.targetDirection.plus(currentDirection.unaryMinus());
    }

    @Override
    public void execute() {
        currentDirection = Rotation2d.fromDegrees(driveTrain.getHeading());
        deltaDirection = this.targetDirection.plus(currentDirection.unaryMinus());
        if (deltaDirection.getSin() > 0) {
            driveTrain.arcadeDrive(0, -0.1);
        } else {
            driveTrain.arcadeDrive(0, 0.1);
        }

    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(deltaDirection.getSin()) < .05 && deltaDirection.getCos()>0;
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(new Translation2d(1,3),Rotation2d.fromDegrees(69));
    }
}

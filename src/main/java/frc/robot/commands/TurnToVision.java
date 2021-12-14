package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.VisionMode;

public class TurnToVision extends CommandBase implements AutoCommandInterface {
    Vision vision;
    DriveTrain driveTrain;
    Rotation2d currentDirection;
    Rotation2d deltaDirection;

    public TurnToVision(Vision vision, DriveTrain driveTrain) {
        this.vision = vision;
        this.driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        vision.setLedRing(true);
        vision.setMode(VisionMode.FACEFINDER);
    }

    @Override
    public void execute() {
            driveTrain.arcadeDrive(0, vision.getTurnDirection()*0.1);


    }

    @Override
    public void end(boolean interrupted) {
        vision.setLedRing(false);
        driveTrain.arcadeDrive(0, 0);
        vision.setMode(VisionMode.SHOOTER);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(new Translation2d(1,3),Rotation2d.fromDegrees(69));
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.FieldMapHome;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PowerPort extends SequentialCommandGroup {
    /** Creates a new PowerPort. */
  
    // add a yOffset variable. make simulation testing much easier
    final double yOffset = 10.0;
    final Rotation2d rotation180 = Rotation2d.fromDegrees(180.0);

    // sets the position where balls will be fed into the robot
    private final Pose2d reintroductionPose = new Pose2d(FieldMapHome.gridPoint('C', 11, -10, yOffset), rotation180);

    // sets the position that the robot starts in and shoots in
    private final Pose2d initialPose = new Pose2d(FieldMapHome.gridPoint('C', 2, 0, yOffset), rotation180);

    public PowerPort(DriveTrain robotDrive, Shooter shooter, Carousel carousel, CarouselCommand carouselCommand, Climber climber) {

        double maxSpeed = 1.5;
        double maxAccel = 1.0;

        // This will make the robot slow down around turns. Probably not necessary for this Auto, but can't hurt.
        TrajectoryConstraint centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(1.5);

        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics,
                10);
        // sets the constraints for forwards and backwards driving
        TrajectoryConfig configForward = new TrajectoryConfig(maxSpeed, maxAccel)
            .setKinematics(Constants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint)
            .addConstraint(centripetalAccelerationConstraint)
            .setReversed(false);
        TrajectoryConfig configBackward = new TrajectoryConfig(maxSpeed, maxAccel)
            .setKinematics(Constants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint)
            .addConstraint(centripetalAccelerationConstraint)
            .setReversed(true);

        RamseteCommand ramsetebackward = createRamseteCommand(initialPose, reintroductionPose, configBackward, robotDrive);
        RamseteCommand ramseteforward = createRamseteCommand(reintroductionPose, initialPose, configForward, robotDrive); 

        // robot must start with the intake up so it must be lowered before shooting
        addCommands(
            new ParallelCommandGroup(new DeployIntake(climber)),
            new TurnShootTurnBack(robotDrive, shooter, carousel, carouselCommand, null, 180)
        );

        // drives back to get balls and then forward to shoot 5 times
        for (int i = 0; i < 5; i++) {
            addCommands(
                ramsetebackward,
                new HardStop(robotDrive),
                new WaitForFullCarousel(carousel),

                ramseteforward,
                new HardStop(robotDrive),
                new TurnShootTurnBack(robotDrive, shooter, carousel, carouselCommand, null, 180)
            );
        }
    }

    RamseteCommand createRamseteCommand(Pose2d start, Pose2d end, TrajectoryConfig config, DriveTrain robotDrive) {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            start,
            List.of(),
            end,
            config);
        return new RamseteCommand(
            trajectory, 
            robotDrive::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(Constants.ksVolts,
                                    Constants.kvVoltSecondsPerMeter,
                                    Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            robotDrive::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            robotDrive::tankDriveVolts,
            robotDrive
        );
    }
}

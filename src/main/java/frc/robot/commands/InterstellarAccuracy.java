package frc.robot.commands;

import java.util.List;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldMapHome;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class InterstellarAccuracy extends SequentialCommandGroup implements AutoCommandInterface {
    
    final Rotation2d rotation180 = Rotation2d.fromDegrees(180.0);
    
    // add a yOffset variable. make simulation testing much easier
    final double yOffset = 10.0;

    // Define the initial pose to be used by this command. This will be used in the initial trajectory
    // and will allow the system to query for it
    private final Pose2d reintroductionPose = new Pose2d(FieldMapHome.gridPoint('C', 11, -10, yOffset), rotation180);

    // Center of robot is 69" from wall
    private final Pose2d initialPose = new Pose2d(FieldMapHome.gridPoint('C', 2, 0, yOffset), rotation180);
    
    public InterstellarAccuracy(DriveTrain robotDrive, Shooter shooter, Carousel carousel, CarouselCommand carouselCommand, Climber climber) {
        SmartDashboard.putBoolean("Interstellar", false);
        
        double maxSpeed = 1.5;
        double maxAccel = 1.0;

        // This will make the robot slow down around turns. Probably not necessary for this Auto, but can't hurt.
        TrajectoryConstraint centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(1.5);

        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics,
                10);
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

        RamseteCommand ramsete1backward = createRamseteCommand(initialPose, reintroductionPose, configBackward, robotDrive);
        RamseteCommand ramsete2forward = createRamseteCommand(reintroductionPose, new Pose2d(FieldMapHome.gridPoint('C', 4, 0, yOffset), rotation180), configForward, robotDrive);
        RamseteCommand ramsete2backward = createRamseteCommand(new Pose2d(FieldMapHome.gridPoint('C', 4, 0, yOffset), rotation180), reintroductionPose, configBackward, robotDrive);
        RamseteCommand ramsete3forward = createRamseteCommand(reintroductionPose, new Pose2d(FieldMapHome.gridPoint('C', 6, 0, yOffset), rotation180), configForward, robotDrive);
        RamseteCommand ramsete3backward = createRamseteCommand(new Pose2d(FieldMapHome.gridPoint('C', 6, 0, yOffset), rotation180), reintroductionPose, configBackward, robotDrive);
        RamseteCommand ramsete4forward = createRamseteCommand(reintroductionPose, new Pose2d(FieldMapHome.gridPoint('C', 8, 0, yOffset), rotation180), configForward, robotDrive);
        RamseteCommand ramsete4backward = createRamseteCommand(new Pose2d(FieldMapHome.gridPoint('C', 8, 0, yOffset), rotation180), reintroductionPose, configBackward, robotDrive);
        RamseteCommand ramsete1forward2 = createRamseteCommand(reintroductionPose, new Pose2d(FieldMapHome.gridPoint('C', 8, 0, yOffset), rotation180), configForward, robotDrive);

        addCommands(
            new ParallelCommandGroup(new DeployIntake(climber),
                        new TurnShootTurnBack(robotDrive, shooter, carousel, carouselCommand, null, 180)),
            ramsete1backward.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            new WaitForFullCarousel(carousel),

            ramsete2forward.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            new TurnShootTurnBack(robotDrive, shooter, carousel, carouselCommand, null, 180),
            ramsete2backward.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            new WaitForFullCarousel(carousel),

            ramsete3forward.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            new TurnShootTurnBack(robotDrive, shooter, carousel, carouselCommand, null, 180),
            ramsete3backward.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            new WaitForFullCarousel(carousel),

            ramsete4forward.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            new TurnShootTurnBack(robotDrive, shooter, carousel, carouselCommand, null, 180),
            ramsete4backward.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            new WaitForFullCarousel(carousel),

            ramsete1forward2.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            new TurnShootTurnBack(robotDrive, shooter, carousel, carouselCommand, null, 180)
        );
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

    public void end(boolean interrupted){
        SmartDashboard.putBoolean("InterstellarAccuracy", interrupted);
        System.out.println("InterstellarAccuracy ended. interrupted = " + interrupted);
    }

    // Allows the system to get the initial pose of this command
    public Pose2d getInitialPose() {
        return initialPose;
    }
}



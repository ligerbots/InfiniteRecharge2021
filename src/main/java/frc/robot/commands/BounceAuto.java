package frc.robot.commands;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldMapHome;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;

public class BounceAuto extends SequentialCommandGroup implements AutoCommandInterface {

    final Rotation2d rotation180 = Rotation2d.fromDegrees(180.0);
    final Rotation2d rotation270 = Rotation2d.fromDegrees(270.0);
    final Rotation2d rotation90 = Rotation2d.fromDegrees(90.0);
    
    // Define the initial pose to be used by this command. This will be used in the initial trajectory
    // and will allow the system to query for it
    private Pose2d initialPose = new Pose2d(FieldMapHome.gridPoint('C', 1), rotation180);

    public BounceAuto(DriveTrain robotDrive, Climber climber) {

        TrajectoryConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter), 
            Constants.kDriveKinematics, 10);


        double maxSpeed = 3.0;
        double maxAccel = 3.0;

        TrajectoryConstraint centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(2);

        TrajectoryConfig configForward = new TrajectoryConfig(maxSpeed, maxAccel)
                .setKinematics(Constants.kDriveKinematics)
                    .addConstraint(autoVoltageConstraint)
                    .addConstraint(centripetalAccelerationConstraint);

        TrajectoryConfig configBackward = new TrajectoryConfig(maxSpeed, maxAccel)
                .setKinematics(Constants.kDriveKinematics)
                    .addConstraint(autoVoltageConstraint)
                    .addConstraint(centripetalAccelerationConstraint)
                    .setReversed(true);
      
        Trajectory backTrajectory1 = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                initialPose,
                List.of( 
                    FieldMapHome.gridPoint('C', 2, 20, 15)
                ),
                new Pose2d(FieldMapHome.gridPoint('A', 3, 0, -45.0/2), rotation270),
                configBackward);

        Trajectory forwardTrajectory1 = TrajectoryGenerator.generateTrajectory(

                new Pose2d(FieldMapHome.gridPoint('A', 3, 0, -45.0/2), rotation270),
                List.of(
                    FieldMapHome.gridPoint('C', 4, -20, 0),
                    FieldMapHome.gridPoint('E', 5, 5, 0)
                ) ,
                new Pose2d(FieldMapHome.gridPoint('A', 6, 0, -10), rotation90),
                configForward);

        Trajectory backTrajectory2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(FieldMapHome.gridPoint('A', 6, 0, -10), rotation90),
                List.of(
                        FieldMapHome.gridPoint('D', 7, -27, 0),
                        FieldMapHome.gridPoint('E', 7, 0, 5),
                        FieldMapHome.gridPoint('E', 8, 0, 5),
                        FieldMapHome.gridPoint('D', 9, -3, 0)
                ),
                new Pose2d(FieldMapHome.gridPoint('A', 9, 5, -35.0/2), rotation270),
                configBackward);

        Trajectory forwardTrajectory2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(FieldMapHome.gridPoint('A', 9, 5, -35.0/2), rotation270),
                List.of(
                        FieldMapHome.gridPoint('B', 10, -20, -20)
                ),
                new Pose2d(FieldMapHome.gridPoint('C', 11, 0, 10), new Rotation2d(0)),
                configForward);
                  
        System.out.println("DEBUG: Bounce path");
        System.out.print("DEBUG: maxSpeed = " + maxSpeed + " maxAcceleration = " + maxAccel + " ");
        double pathTime = backTrajectory1.getTotalTimeSeconds() + backTrajectory2.getTotalTimeSeconds()
            + forwardTrajectory1.getTotalTimeSeconds() + forwardTrajectory2.getTotalTimeSeconds();
        System.out.println("Path time = " + pathTime);
        if (Robot.isSimulation()) {
            TrajectoryWriter writer = new TrajectoryWriter("Bounce");
            writer.WriteTrajectory(backTrajectory1);
            writer.WriteTrajectory(forwardTrajectory1);
            writer.WriteTrajectory(backTrajectory2);
            writer.WriteTrajectory(forwardTrajectory2);
        }

        RamseteCommand ramseteBackward1 = new RamseteCommand(
            backTrajectory1,
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

        RamseteCommand ramseteForward1 = new RamseteCommand(
            forwardTrajectory1,
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

        RamseteCommand ramseteBackward2 = new RamseteCommand(
            backTrajectory2,
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

        RamseteCommand ramseteForward2 = new RamseteCommand(
            forwardTrajectory2,
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
        
        addCommands(            
            new ParallelCommandGroup(new DeployIntake(climber),ramseteBackward1).andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            ramseteForward1.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            ramseteBackward2.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            ramseteForward2.andThen(() -> robotDrive.tankDriveVolts(0, 0))
        );
    }

    // Allows the system to get the initial pose of this command
    public Pose2d getInitialPose() {
        return initialPose;
    }
}

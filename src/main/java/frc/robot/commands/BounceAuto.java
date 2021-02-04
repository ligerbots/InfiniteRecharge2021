package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldMapHome;
import frc.robot.subsystems.DriveTrain;
public class BounceAuto extends SequentialCommandGroup implements AutoCommandInterface {
  Rotation2d rotation1 = Rotation2d.fromDegrees(180.0);
  Rotation2d rotation2 = Rotation2d.fromDegrees(270.0);
  Rotation2d rotation3 = Rotation2d.fromDegrees(90.0);
    // Define the initial pose to be used by this command. This will be used in the initial trajectory
    // and will allow the system to query for it
    private Pose2d initialPose = new Pose2d(FieldMapHome.gridPoint('C', 1), rotation1);

    public BounceAuto(DriveTrain robotDrive, DriveCommand drivecommand) {
        drivecommand.cancel();

        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics,
                10);
        TrajectoryConfig configForward = new TrajectoryConfig(SmartDashboard.getNumber("AutoMaxSpeed",1.75), SmartDashboard.getNumber("AutoMaxAcceleration",1.5))
                .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint);

        TrajectoryConfig configBackward = new TrajectoryConfig(SmartDashboard.getNumber("AutoMaxSpeed",1.75), SmartDashboard.getNumber("AutoMaxAcceleration",1.5))
                .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint).setReversed(true);


      
        Trajectory backTrajectory1 = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                initialPose,
                List.of( 
                    FieldMapHome.gridPoint('B', 3)
                ),
                new Pose2d(FieldMapHome.gridPoint('A', 3, 0, -37.5/2), rotation2),
                configBackward);

        Trajectory forwardTrajectory1 = TrajectoryGenerator.generateTrajectory(

                new Pose2d(FieldMapHome.gridPoint('A', 3, 0, -37.5/2), rotation2),
                List.of(
                    FieldMapHome.gridPoint('C', 4, -15, 0),
                    FieldMapHome.gridPoint('E', 5)
                ) ,
                new Pose2d(FieldMapHome.gridPoint('A', 6, 0, -37.5/2), rotation3),
                configForward);

        Trajectory backTrajectory2 = TrajectoryGenerator.generateTrajectory(

          new Pose2d(FieldMapHome.gridPoint('A', 6, 0, -37.5/2), rotation3),
          List.of( 
              FieldMapHome.gridPoint('D', 7, -20, 0),
              FieldMapHome.gridPoint('E', 7, 0, 10),
              FieldMapHome.gridPoint('E', 8, 0, 10),
              FieldMapHome.gridPoint('D', 9, -10, 0)         
          ),
          new Pose2d(FieldMapHome.gridPoint('A', 9, 0, -37.5/2), rotation2),
          configBackward);

        Trajectory forwardTrajectory2 = TrajectoryGenerator.generateTrajectory(

        new Pose2d(FieldMapHome.gridPoint('A', 9, 0, -37.5/2), rotation2),
        List.of(
            FieldMapHome.gridPoint('B', 10, -20, -20)
        ) ,
        new Pose2d(FieldMapHome.gridPoint('C', 11), new Rotation2d(0)),
        configForward);       
                  

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
            ramseteBackward1.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
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

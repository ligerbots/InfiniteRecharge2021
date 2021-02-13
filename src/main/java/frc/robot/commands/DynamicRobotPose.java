package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.FieldMapHome;
import frc.robot.subsystems.DriveTrain;

public class DynamicRobotPose extends SequentialCommandGroup implements AutoCommandInterface {

    private Pose2d initialPose=new Pose2d(2,2,new Rotation2d(0));

    public DynamicRobotPose(DriveTrain robotDrive, DriveCommand drivecommand) {
        Rotation2d rotation = Rotation2d.fromDegrees(0.0);
        
       
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics,
                10);


        TrajectoryConfig configForward = new TrajectoryConfig(2.0, 2.0)
                .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint).setReversed(false);

        ArrayList<Translation2d> waypoints=new ArrayList<>();
        waypoints.add(new Translation2d(5,2));
        waypoints.add(new Translation2d(5,3));
        for(int i=0;i<100;i++){
            waypoints.add(new Translation2d(2,3));
            waypoints.add(new Translation2d(2,2));
            waypoints.add(new Translation2d(5,2));
            waypoints.add(new Translation2d(5,3));
        }
        Trajectory backTrajectory = TrajectoryGenerator.generateTrajectory(
                initialPose,
                waypoints,
                new Pose2d(2,3,new Rotation2d(Math.PI)), 
                configForward);


        RamseteCommand ramseteForward = new RamseteCommand(
            backTrajectory,
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
            // Run the backward trajectory and then stop when we get to the end
            ramseteForward.andThen(() -> robotDrive.tankDriveVolts(0, 0))
        );
    }

    // Allows the system to get the initial pose of this command
    public Pose2d getInitialPose() {
        return initialPose;
    }
}
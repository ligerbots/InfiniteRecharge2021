package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.FieldMapHome;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class AutoNavPaths extends SequentialCommandGroup implements AutoCommandInterface {

    public enum Path{
        Barrel, Slalom;
    }

    // Define the initial pose to be used by this command. This will be used in the initial trajectory
    // and will allow the system to query for it
    private Pose2d initialPose;

    public AutoNavPaths(DriveTrain robotDrive, DriveCommand drivecommand, Path autoID) {
        final Rotation2d rotation = Rotation2d.fromDegrees(0.0);
        
        List<Translation2d> waypointList = null;    // set to null to suppress warning
        Pose2d endPose = null;

        switch (autoID) {
            case Barrel:
                initialPose = new Pose2d(FieldMapHome.gridPoint('C', 1, 12.0, 0.0), rotation);
                waypointList = List.of(FieldMapHome.gridPoint('C', 5),
                                       FieldMapHome.gridPoint('D', 6),
                                       FieldMapHome.gridPoint('E', 5, 0, 5),
                                       FieldMapHome.gridPoint('D', 4, -5, 0),
                                       FieldMapHome.gridPoint('C', 5, -10, 0),
                                       FieldMapHome.gridPoint('C', 8),
                                       FieldMapHome.gridPoint('B', 9),
                                       FieldMapHome.gridPoint('A', 8),
                                       FieldMapHome.gridPoint('B', 7),
                                       FieldMapHome.gridPoint('D', 8, 0, -10),
                                       FieldMapHome.gridPoint('E', 10, 0, 3),
                                       FieldMapHome.gridPoint('D', 11, -1, 0),
                                       //FieldMapHome.gridPoint('C', 11, -7, 0),
                                       FieldMapHome.gridPoint('C', 9, 10, 1)
                                       );
                endPose = new Pose2d(FieldMapHome.gridPoint('C', 2), new Rotation2d(Math.PI));
                break;

            case Slalom:
                initialPose = new Pose2d(FieldMapHome.gridPoint('E', 1, 5, 0), rotation);
                waypointList = List.of(
                                       FieldMapHome.gridPoint('C', 4),
                                       FieldMapHome.gridPoint('C', 8),
                                       FieldMapHome.gridPoint('D', 9),
                                       FieldMapHome.gridPoint('E', 10),
                                       FieldMapHome.gridPoint('D', 11),
                                       FieldMapHome.gridPoint('C', 10),
                                       FieldMapHome.gridPoint('E', 8),
                                       FieldMapHome.gridPoint('E', 7),
                                       FieldMapHome.gridPoint('E', 4, 7, 0));
                endPose = new Pose2d(FieldMapHome.gridPoint('C', 2), Rotation2d.fromDegrees(135.0));
                break;
        }

        TrajectoryConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics,
                10);

        // Initial baseline
        double maxSpeed = 2.0;
        double maxAccel = 2.0;
        TrajectoryConfig configForward = new TrajectoryConfig(maxSpeed, maxAccel)
                .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint).setReversed(false);

        Trajectory forwardTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                initialPose,
                waypointList, 
                endPose, 
                configForward);

        if (Robot.isSimulation()) {
            System.out.println("DEBUG: AutoNav path " + autoID.name());
            System.out.println("DEBUG: maxSpeed = " + maxSpeed + " maxAcceleration = " + maxAccel);
            // for (State state : backTrajectory.getStates()) {
            // System.out.println("DEBUG: backTrajectory STATE "+ state.poseMeters);
            // }
            System.out.println("DEBUG: AutoNav path time = " + forwardTrajectory.getTotalTimeSeconds());
            TrajectoryWriter writer = new TrajectoryWriter(autoID.name());
            writer.WriteTrajectory(forwardTrajectory);
            writer.WriteWaypoints(initialPose, waypointList, endPose);
        }

        RamseteCommand ramseteForward = new RamseteCommand(
            forwardTrajectory,
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
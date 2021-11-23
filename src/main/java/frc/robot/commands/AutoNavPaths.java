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
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
// import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.FieldMapHome;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;

public class AutoNavPaths extends SequentialCommandGroup implements AutoCommandInterface {

    public enum Path{
        Circle, Albert;
    }

    // Define the initial pose to be used by this command. This will be used in the initial trajectory
    // and will allow the system to query for it
    private Pose2d initialPose;
    private List<Translation2d> waypointList = null;    // set to null to suppress warning
    private Trajectory forwardTrajectory;

    public AutoNavPaths(DriveTrain robotDrive, Path autoID) {
        final Rotation2d rotation = Rotation2d.fromDegrees(0.0);
        
        Pose2d endPose = null;

        // Define these here, but we may override them within the case statement so we can tune each
        // path individually
        double maxSpeed = 1.5;
        double maxAccel = 1.0;
        double maxCentripetal = 1.5;
        // This will make the robot slow down around turns

        switch (autoID) {

                case Circle:

                maxSpeed = 1;
                maxAccel = 1;
                maxCentripetal = 1;

                initialPose = new Pose2d(FieldMapHome.gridpointPolar('D', 5, 30, 0), Rotation2d.fromDegrees(90.0));
                waypointList = List.of(FieldMapHome.gridpointPolar('D',5, 30,90),
                                        FieldMapHome.gridpointPolar('D',5,30,180),
                                        FieldMapHome.gridpointPolar('D',5,30,270)
                                        
                                       );
                endPose = new Pose2d(FieldMapHome.gridpointPolar('D', 5, 30, 0), Rotation2d.fromDegrees(90.0));
                break;

            case Albert:

                // Set speed and acceleration values
                maxSpeed = 6.5;
                maxAccel = 3.2;
                maxCentripetal = 3.8;

                initialPose = new Pose2d(FieldMapHome.gridPoint('D', 1, 5, 0), rotation);
                waypointList = List.of(FieldMapHome.gridPoint('C',1),
                                        FieldMapHome.gridPoint('C',3, -3,-3),
                                        FieldMapHome.gridPoint('B',2,3,0)
                                        
                                       );
                endPose = new Pose2d(FieldMapHome.gridPoint('C', 2, -15, 30), Rotation2d.fromDegrees(150.0));
                break;

        }

        TrajectoryConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics,
                10);

        TrajectoryConstraint centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(maxCentripetal);

        TrajectoryConfig configForward = new TrajectoryConfig(maxSpeed, maxAccel)
                .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint).addConstraint(centripetalAccelerationConstraint).setReversed(false);

        forwardTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                initialPose,
                waypointList, 
                endPose, 
                configForward);

        System.out.println("DEBUG: AutoNav path " + autoID.name());
        System.out.print("DEBUG: maxSpeed = " + maxSpeed + " maxAcceleration = " + maxAccel + " ");
        // for (State state : backTrajectory.getStates()) {
        //     System.out.println("DEBUG: backTrajectory STATE "+ state.poseMeters);
        // }
        System.out.println("Path time = " + forwardTrajectory.getTotalTimeSeconds());
        // if (Robot.isSimulation()) {
        //     TrajectoryWriter writer = new TrajectoryWriter(autoID.name());
        //     writer.WriteTrajectory(forwardTrajectory);
        //     writer.WriteWaypoints(initialPose, waypointList, endPose);
        // }

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
            new ParallelCommandGroup(ramseteForward).andThen(() -> robotDrive.tankDriveVolts(0, 0))
        );
    }

    // Allows the system to get the initial pose of this command
    public Pose2d getInitialPose() {
        return initialPose;
    }

    public void plotTrajectory(TrajectoryPlotter plotter) {
        plotter.plotTrajectory(forwardTrajectory);
        plotter.plotWaypoints(waypointList);
    }
}
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
import frc.robot.Robot;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class GalacticSearchAuto extends SequentialCommandGroup implements AutoCommandInterface {

    public enum Path{
        RedA, RedB, BlueA, BlueB;
    }

    // Define the initial pose to be used by this command. This will be used in the initial trajectory
    // and will allow the system to query for it
    private Pose2d initialPose;

    public GalacticSearchAuto(DriveTrain robotDrive, Carousel carousel, Intake intake, Path autoID, Climber climber) {
        final Rotation2d rotation180 = Rotation2d.fromDegrees(180.0);

        // Need to run the intake and Carousel for picking up the balls
        // Construct them here for inclusion into the command group below.
        CarouselCommand carouselCommand = new CarouselCommand(carousel);
        IntakeCommand intakeCommand = new IntakeCommand(intake, Constants.INTAKE_SPEED);

        List<Translation2d> waypointList = null;    // set to null to suppress warning
        Pose2d endPose = null;

        // Define these here, but we may override them within the case statement so we can tune each
        // path individually
        double maxSpeed = 3.5;
        double maxAccel = 3.5;

        // This will make the robot slow down around turns
        TrajectoryConstraint centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(2);

        switch (autoID) {
            case RedA:
                initialPose = new Pose2d(FieldMapHome.gridPoint('B', 1, 45.0/2, -15), Rotation2d.fromDegrees(153.0));
                waypointList = List.of(FieldMapHome.gridPoint('D', 5, 0, 5),
                                       FieldMapHome.gridPoint('A', 6, 0, -20));
                endPose =  new Pose2d(FieldMapHome.gridPoint('A', 11), rotation180);
                break;

            case RedB:
                initialPose = new Pose2d(FieldMapHome.gridPoint('A', 1, 45.0/2, 0), Rotation2d.fromDegrees(135));
                waypointList = List.of(FieldMapHome.gridPoint('D', 5, 0, 15),
                                       FieldMapHome.gridPoint('B', 7, 0, -15));
                endPose =  new Pose2d(FieldMapHome.gridPoint('B', 11), rotation180);
                break;

            case BlueA:
                initialPose = new Pose2d(FieldMapHome.gridPoint('E', 1, 45.0/2, 0), rotation180);
                waypointList = List.of(FieldMapHome.gridPoint('E', 6, -15, 0),
                                       FieldMapHome.gridPoint('B', 7, 0, -15),
                                       FieldMapHome.gridPoint('C', 9, 0, 10));
                endPose =  new Pose2d(FieldMapHome.gridPoint('C', 11), rotation180);
                break;

            case BlueB:
                initialPose = new Pose2d(FieldMapHome.gridPoint('D', 1, 45.0/2, 0), rotation180);
                waypointList = List.of(FieldMapHome.gridPoint('D', 6, 0, 10),
                                       FieldMapHome.gridPoint('B', 8, 0, -5),
                                       FieldMapHome.gridPoint('D', 10, 0, 10));
                endPose =  new Pose2d(FieldMapHome.gridPoint('D', 11, 0, -10), Rotation2d.fromDegrees(135));
                break;
        }

        TrajectoryConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics,
                10);

        TrajectoryConfig configBackward = new TrajectoryConfig(maxSpeed, maxAccel)
                .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint).addConstraint(centripetalAccelerationConstraint).setReversed(true);

        Trajectory backTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                initialPose,
                waypointList, 
                endPose, 
                configBackward);

        System.out.println("DEBUG: Galactic Search path " + autoID.name());
        System.out.print("DEBUG: maxSpeed = " + maxSpeed + " maxAcceleration = " + maxAccel + " ");
        // for (State state : backTrajectory.getStates()) {
        //     System.out.println("DEBUG: backTrajectory STATE "+ state.poseMeters);
        // }
        System.out.println("Path time = " + backTrajectory.getTotalTimeSeconds());
        if ( Robot.isSimulation() ) {
            TrajectoryWriter writer = new TrajectoryWriter(autoID.name());
            writer.WriteTrajectory(backTrajectory);
            writer.WriteWaypoints(initialPose, waypointList, endPose);
        }

        RamseteCommand ramseteBackward = new RamseteCommand(
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
        if(autoID == Path.RedA|| autoID == Path.RedB){
            addCommands(
                // We need to start both the carousel command and the intake command in parallel
                // with the ramseteBackward command.
                carouselCommand.alongWith(intakeCommand, 
                                          // At the end of the remsete trajectory, stop the motors.
                                          new SequentialCommandGroup(new DeployIntake(climber),ramseteBackward)
                                          .andThen(() -> robotDrive.tankDriveVolts(0, 0)))
            );
        }else{
            addCommands(
                // We need to start both the carousel command and the intake command in parallel
                // with the ramseteBackward command.
                carouselCommand.alongWith(intakeCommand, 
                                        // At the end of the remsete trajectory, stop the motors.
                                        new ParallelCommandGroup(new DeployIntake(climber),ramseteBackward)
                                        .andThen(() -> robotDrive.tankDriveVolts(0, 0)))
            );
        }
    }

    // Allows the system to get the initial pose of this command
    public Pose2d getInitialPose() {
        return initialPose;
    }
}
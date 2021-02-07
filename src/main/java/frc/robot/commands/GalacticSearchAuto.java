package frc.robot.commands;

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
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.FieldMapHome;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class GalacticSearchAuto extends SequentialCommandGroup implements AutoCommandInterface {

    public enum Path {
        RedA, RedB, BlueA, BlueB;
    }

    // Define the initial pose to be used by this command. This will be used in the initial trajectory
    // and will allow the system to query for it
    private Pose2d initialPose;

    public GalacticSearchAuto(DriveTrain robotDrive, DriveCommand drivecommand, Carousel carousel, Intake intake, Path autoID) {
        final Rotation2d rotation180 = Rotation2d.fromDegrees(180.0);
        
        //Need to run the intake and Carousel for picking up the balls
        // Construct them here for inclusion into the command group below.
        CarouselCommand carouselCommand = new CarouselCommand(carousel);
        IntakeCommand intakeCommand = new IntakeCommand(intake, Constants.INTAKE_SPEED);

        List<Translation2d> waypointList = null;    // initialize with null to remove warning
        Pose2d endPose = null;

        switch (autoID) {
            case RedA:
                initialPose = new Pose2d(FieldMapHome.gridPoint('B', 1, 45/2,  -15), Rotation2d.fromDegrees(153.0));
                waypointList = List.of(FieldMapHome.gridPoint('D', 5, 0 , 5),
                                       FieldMapHome.gridPoint('A', 6 , 0 , -20));
                endPose =  new Pose2d(FieldMapHome.gridPoint('A', 11), rotation180);
                break;

            case RedB:
                // Test case. Try starting from A1
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
                                       FieldMapHome.gridPoint('B', 8 ,0 ,-5),
                                       FieldMapHome.gridPoint('D', 10, 0, 10));
                endPose =  new Pose2d(FieldMapHome.gridPoint('D', 11, 0, -10), Rotation2d.fromDegrees(135));
                break;
        }

        TrajectoryConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics,
                10);

        // Initial baseline
        double maxSpeed = SmartDashboard.getNumber("AutoMaxSpeed", 2.0);
        double maxAccel = SmartDashboard.getNumber("AutoMaxAcceleration", 2.0);
        TrajectoryConfig configBackward = new TrajectoryConfig(maxSpeed, maxAccel)
                .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint).setReversed(true);

        Trajectory backTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                initialPose,
                waypointList, 
                endPose, 
                configBackward);

        System.out.println("DEBUG: Galactic Search path " + autoID.name());
        System.out.println("DEBUG: maxSpeed = " + maxSpeed + " maxAcceleration = " + maxAccel);
        // for (State state : backTrajectory.getStates()) {
        //     System.out.println("DEBUG: backTrajectory STATE "+ state.poseMeters);
        // }
        System.out.println("DEBUG: Galactic Search path time = " + backTrajectory.getTotalTimeSeconds());

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

        addCommands(
            // We need to start both the carousel command and the intake command in parallel
            // with the ramseteBackward command.
            carouselCommand.alongWith(intakeCommand, 
                                      // At the end of the remsete trajectory, stop the motors.
                                      ramseteBackward.andThen(() -> robotDrive.tankDriveVolts(0, 0)))
        );
    }

    // Allows the system to get the initial pose of this command
    public Pose2d getInitialPose() {
        return initialPose;
    }
}
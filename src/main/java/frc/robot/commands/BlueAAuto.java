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
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldMapHome;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
public class BlueAAuto extends SequentialCommandGroup implements AutoCommandInterface {

    // Define the initial pose to be used by this command. This will be used in the initial trajectory
    // and will allow the system to query for it
    private Pose2d initialPose = new Pose2d(30.0 * Constants.inchToMetersConversionFactor, 30.0 * Constants.inchToMetersConversionFactor, new Rotation2d(0.0) );

    public BlueAAuto(DriveTrain robotDrive, Intake intake, DriveCommand drivecommand, Carousel carousel) {
        //Don't want the Human player to have input to the robot 
        drivecommand.cancel(); 
        //Run the intake and Carousel for picking up the balls
        intake.run(0.4);
        carousel.spin(Constants.CAROUSEL_INTAKE_SPEED);
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics,
                10);

        TrajectoryConfig configBackward = new TrajectoryConfig(SmartDashboard.getNumber("AutoMaxSpeed",1.75), SmartDashboard.getNumber("AutoMaxAcceleration",1.5))
                .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint).setReversed(true);

        Trajectory backTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                initialPose,
                List.of( 
                    //TODO: NEED WAYPOINTS
                    FieldMapHome.gridPoint('E', 6),
                    FieldMapHome.gridPoint('B', 7),   
                    FieldMapHome.gridPoint('C', 9)             
                ), 
                new Pose2d(FieldMapHome.gridPoint('C', 11), new Rotation2d(0.0)),
                configBackward);

        for (State state : backTrajectory.getStates()) {
            System.out.println("DEBUG: backTrajectory STATE "+ state.poseMeters);
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
        addCommands(
            ramseteBackward.andThen(() -> robotDrive.tankDriveVolts(0, 0))
        );
    }

    // Allows the system to get the initial pose of this command
    public Pose2d getInitialPose() {
        return initialPose;
    }
}
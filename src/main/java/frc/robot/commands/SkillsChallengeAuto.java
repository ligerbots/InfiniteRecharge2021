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
import frc.robot.subsystems.DriveTrain;
public class SkillsChallengeAuto extends SequentialCommandGroup implements AutoCommandInterface {

    // Define the initial pose to be used by this command. This will be used in the initial trajectory
    // and will allow the system to query for it
    public enum Path{
        RedA, RedB, BlueA, BlueB, Barrel, Slalom;
    }

    private Pose2d initialPose;

    public SkillsChallengeAuto(DriveTrain robotDrive, DriveCommand drivecommand, Path autoID) {
        drivecommand.cancel(); 
        Rotation2d rotation = Rotation2d.fromDegrees(180.0);
        //Run the intake and Carousel for picking up the balls
        List<Translation2d> waypointList;
        Pose2d endPose;
        switch(autoID){
            case RedA:
                initialPose = new Pose2d(FieldMapHome.gridPoint('C', 1), rotation);
                waypointList = List.of(FieldMapHome.gridPoint('C', 3),
                                       FieldMapHome.gridPoint('D', 5),
                                       FieldMapHome.gridPoint('A', 6));
                endPose =  new Pose2d(FieldMapHome.gridPoint('B', 11), rotation);
                break;
            case RedB:
                initialPose = new Pose2d(FieldMapHome.gridPoint('B', 1), rotation);
                waypointList = List.of(FieldMapHome.gridPoint('B', 3),
                                       FieldMapHome.gridPoint('D', 5),
                                       FieldMapHome.gridPoint('B', 7));
                endPose =  new Pose2d(FieldMapHome.gridPoint('B', 11), rotation);
                break;
            case BlueA:
                initialPose = new Pose2d(FieldMapHome.gridPoint('E', 1), rotation);
                waypointList = List.of(FieldMapHome.gridPoint('E', 6),
                                       FieldMapHome.gridPoint('B', 7),
                                       FieldMapHome.gridPoint('C', 9));
                endPose =  new Pose2d(FieldMapHome.gridPoint('C', 11), rotation);
                break;
            case BlueB:
                initialPose = new Pose2d(FieldMapHome.gridPoint('D', 1), rotation);
                waypointList = List.of(FieldMapHome.gridPoint('D', 6),
                                       FieldMapHome.gridPoint('B', 8),
                                       FieldMapHome.gridPoint('D', 10));
                endPose =  new Pose2d(FieldMapHome.gridPoint('E', 11), rotation);
                break;
            case Barrel:
                initialPose = new Pose2d(FieldMapHome.gridPoint('C', 2), rotation);
                waypointList = List.of(FieldMapHome.gridPoint('C', 4),
                                       FieldMapHome.gridPoint('D', 6),
                                       FieldMapHome.gridPoint('E', 5),
                                       FieldMapHome.gridPoint('D', 4),
                                       FieldMapHome.gridPoint('C', 5),
                                       FieldMapHome.gridPoint('B', 9),
                                       FieldMapHome.gridPoint('A', 8),
                                       FieldMapHome.gridPoint('B', 7),
                                       FieldMapHome.gridPoint('D', 8),
                                       FieldMapHome.gridPoint('E', 10),
                                       FieldMapHome.gridPoint('D', 11),
                                       FieldMapHome.gridPoint('C', 10));
                endPose = new Pose2d(FieldMapHome.gridPoint('C', 2), new Rotation2d(0.0));
                break;
            case Slalom:
                //break;
            default:
                waypointList = List.of();
                endPose = new Pose2d();
                initialPose = new Pose2d();
                break;
        }

        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics,
                10);

        TrajectoryConfig configBackward = new TrajectoryConfig(SmartDashboard.getNumber("AutoMaxSpeed",1.75), SmartDashboard.getNumber("AutoMaxAcceleration",1.5))
                .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint).setReversed(true);

        Trajectory backTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                initialPose,
                waypointList, 
                endPose, 
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
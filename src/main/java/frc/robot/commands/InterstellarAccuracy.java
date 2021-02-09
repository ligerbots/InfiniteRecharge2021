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
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldMapHome;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
public class InterstellarAccuracy extends SequentialCommandGroup implements AutoCommandInterface {
  Rotation2d rotation180 = Rotation2d.fromDegrees(180.0);
  Rotation2d rotation270 = Rotation2d.fromDegrees(270.0);
  Rotation2d rotation90 = Rotation2d.fromDegrees(90.0);
    // Define the initial pose to be used by this command. This will be used in the initial trajectory
    // and will allow the system to query for it
    private Pose2d initialPose = new Pose2d(FieldMapHome.gridPoint('C', 11), rotation180);

    public InterstellarAccuracy(DriveTrain robotDrive, DriveCommand drivecommand, Shooter shooter, Carousel carousel, CarouselCommand carouselCommand) {
        drivecommand.cancel();

        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics,
                10);
        TrajectoryConfig configForward = new TrajectoryConfig(SmartDashboard.getNumber("AutoMaxSpeed",1.75), SmartDashboard.getNumber("AutoMaxAcceleration",1.5))
                .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint);
        TrajectoryConfig configBackward = new TrajectoryConfig(SmartDashboard.getNumber("AutoMaxSpeed",1.75), SmartDashboard.getNumber("AutoMaxAcceleration",1.5))
                .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint).setReversed(true);

        Trajectory trajectory1Forward = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                initialPose,
                List.of( 
                ),
                new Pose2d(FieldMapHome.gridPoint('C', 9), rotation180),
                configForward);
        System.out.println(new Pose2d(FieldMapHome.gridPoint('C', 9), rotation180));
        System.out.println(initialPose);
        Trajectory trajectory1Backward = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(FieldMapHome.gridPoint('C', 9), rotation180),
                List.of( 
                ),
                initialPose,
                configBackward);       
                
        Trajectory trajectory2Forward = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                initialPose,
                List.of( 
                ),
                new Pose2d(FieldMapHome.gridPoint('C', 7), rotation180),
                configForward);

        Trajectory trajectory2Backward = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(FieldMapHome.gridPoint('C', 7), rotation180),
                List.of( 
                ),
                initialPose,
                configBackward);      
                  
        Trajectory trajectory3Forward = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                initialPose,
                List.of( 
                ),
                new Pose2d(FieldMapHome.gridPoint('C', 5), rotation180),
                configForward);
    
        Trajectory trajectory3Backward = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(FieldMapHome.gridPoint('C', 5), rotation180),
                List.of( 
                ),
                initialPose,
                configBackward);      

        Trajectory trajectory4Forward = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                initialPose,
                List.of( 
                ),
                new Pose2d(FieldMapHome.gridPoint('C', 3), rotation180),
                configForward);
        
        Trajectory trajectory4Backward = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(FieldMapHome.gridPoint('C', 3), rotation180),
                List.of( 
                ),
                initialPose,
                configBackward);      
        Trajectory trajectory1Forward2 = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                initialPose,
                List.of( 
                ),
                new Pose2d(FieldMapHome.gridPoint('C', 9), rotation180),
                configForward);
        Trajectory trajectory1Backward2 = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(FieldMapHome.gridPoint('C', 9), rotation180),
                List.of( 
                ),
                initialPose,
                configBackward);
    
        RamseteCommand ramsete1forward = createRamsetteCommand(trajectory1Forward, robotDrive);
        RamseteCommand ramsete1backward = createRamsetteCommand(trajectory1Backward, robotDrive);
        RamseteCommand ramsete2forward = createRamsetteCommand(trajectory2Forward, robotDrive);
        RamseteCommand ramsete2backward = createRamsetteCommand(trajectory2Backward, robotDrive);
        RamseteCommand ramsete3forward = createRamsetteCommand(trajectory3Forward, robotDrive);
        RamseteCommand ramsete3backward = createRamsetteCommand(trajectory3Backward, robotDrive);
        RamseteCommand ramsete4forward = createRamsetteCommand(trajectory4Forward, robotDrive);
        RamseteCommand ramsete4backward = createRamsetteCommand(trajectory4Backward, robotDrive);
        RamseteCommand ramsete1forward2 = createRamsetteCommand(trajectory1Forward2, robotDrive);
        RamseteCommand ramsete1backward2 = createRamsetteCommand(trajectory1Backward2, robotDrive);
        // System.out.println("DEBUG: Bounce path");
        // double pathTime = backTrajectory1.getTotalTimeSeconds() + backTrajectory2.getTotalTimeSeconds()
        //     + forwardTrajectory1.getTotalTimeSeconds() + forwardTrajectory2.getTotalTimeSeconds();
        // System.out.println("DEBUG: Bounce path time = " + pathTime);
        
        addCommands(
            ramsete1forward.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            //new ShooterCommand(shooter, carousel, robotDrive, carouselCommand, false),
            ramsete1backward.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            new WaitForSmartDashboard(),

            ramsete2forward.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            //new ShooterCommand(shooter, carousel, robotDrive, carouselCommand, false),
            ramsete2backward.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            new WaitForSmartDashboard(),

            ramsete3forward.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            //new ShooterCommand(shooter, carousel, robotDrive, carouselCommand, false),
            ramsete3backward.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            new WaitForSmartDashboard(),

            ramsete4forward.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            //new ShooterCommand(shooter, carousel, robotDrive, carouselCommand, false),
            ramsete4backward.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            new WaitForSmartDashboard(),

            ramsete1forward2.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            //new ShooterCommand(shooter, carousel, robotDrive, carouselCommand, false),
            ramsete1backward2.andThen(() -> robotDrive.tankDriveVolts(0, 0))
        );
    }
    RamseteCommand createRamsetteCommand(Trajectory trajectory, DriveTrain robotDrive) {
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
    // Allows the system to get the initial pose of this command
    public Pose2d getInitialPose() {
        return initialPose;
    }
    static public class WaitForSmartDashboard extends CommandBase {
        static NetworkTableEntry ballsLoadedEntry;
        static public void initSmartDashboard() {
            ShuffleboardTab tab =Shuffleboard.getTab("Interstellar Accurracy Shooter");
            ballsLoadedEntry=tab.add("Balls loaded", false).withWidget("Toggle Button").getEntry();    
        }
        public WaitForSmartDashboard() {}
        @Override
        public void initialize() {
            ballsLoadedEntry.setBoolean(false);
        }
        @Override
        public void end(boolean interrupted) {
            ballsLoadedEntry.setBoolean(false);
        }
        @Override
        public boolean isFinished() {
            return(ballsLoadedEntry.getBoolean(false));
        }
    }
    
}



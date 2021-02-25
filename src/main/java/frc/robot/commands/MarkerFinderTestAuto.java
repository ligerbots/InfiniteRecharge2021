package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldMapHome;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.VisionMode;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

public class MarkerFinderTestAuto extends SequentialCommandGroup implements AutoCommandInterface {
    
    private Pose2d initialPose = new Pose2d(0,0, new Rotation2d(0));
    Translation2d targetPosition = FieldMapHome.gridPoint('C', 6);
    Vision vision;
    public MarkerFinderTestAuto(DriveTrain robotDrive, DriveCommand drivecommand, Vision vision) {
       this.vision=vision;
       WaitForVision visionWait = new WaitForVision();
       addCommands(
        visionWait.andThen(()->{
            robotDrive.setPose(visionWait.visionResult);
            Translation2d rel = targetPosition.minus(robotDrive.getPose().getTranslation());
            double angle = Units.radiansToDegrees(Math.atan2(rel.getY(), rel.getX()));
            TurnToHeading turn =  new TurnToHeading(robotDrive, angle, 1.0);

            turn.andThen(()->{
                TrajectoryConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
                        Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics,
                        10);

                TrajectoryConfig config = new TrajectoryConfig(1.75, 1.5)
                        .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint);

                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                        robotDrive.getPose(),
                        List.of(),
                        new Pose2d(targetPosition,new Rotation2d(Units.degreesToRadians(angle))), 
                        config);

                RamseteCommand ramsete = new RamseteCommand(
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
                ramsete.andThen(()->{
                    robotDrive.tankDriveVolts(0, 0);
                    TurnToHeading turnback =  new TurnToHeading(robotDrive, 0, 1.0);
                    turnback.schedule();
                }).schedule();
            }).schedule();
        }));
    }

    public void end(boolean interrupted){
    }

    // Allows the system to get the initial pose of this command
    public Pose2d getInitialPose() {
        return initialPose;
    }

    public class WaitForVision extends CommandBase {
        Pose2d visionResult;
        public WaitForVision() {}
        @Override
        public void initialize() {
            visionResult=null;
            vision.setMode(VisionMode.MARKERFINDER_INTAKE);
        }
        
        @Override
        public void execute() {
            visionResult=vision.getMarkerfinderPosition();
        }
        @Override
        public void end(boolean interrupted) {
            vision.setMode(VisionMode.INTAKE);
        }
        @Override
        public boolean isFinished() {
            return(visionResult!=null);
        }
    }
}
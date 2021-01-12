package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;


public class TestPrecisionCommand extends CommandBase implements AutoCommandInterface{
    static Pose2d simulationInitialPose=new Pose2d(feetToMeters(10),feetToMeters(3),new Rotation2d(Math.PI/2));
    static float feetRunway=10;
    static float feetRobotWidth=27.f/12.f;
    static float feetRobotHeight =28.f/12.f;
    static float maxSpeed= (float) Constants.kMaxSpeed;
    static float maxAccel= (float) Constants.kMaxAcceleration;
    static float maxVoltage=10;
    static float innerRadiusFeet=1;
    DriveTrain robotDrive;
    SequentialCommandGroup  command;

    static float feetToMeters(float feet){
        return((float)(feet*12*Constants.inchToMetersConversionFactor));
    }
    static ArrayList<Translation2d> generatePath(float innerRadiusFeet){
        ArrayList<Translation2d> path=new ArrayList<>();
        Translation2d turnEntry=simulationInitialPose.getTranslation().plus(new Translation2d(0,feetToMeters(feetRunway)));
        path.add(turnEntry);

        Translation2d turnEntryExitTopLeft=turnEntry.plus(new Translation2d(-feetToMeters(feetRobotWidth)/2,feetToMeters(feetRobotHeight)/2));
        Translation2d turnCenter=turnEntryExitTopLeft.plus(new Translation2d(-feetToMeters(innerRadiusFeet),feetToMeters(innerRadiusFeet)));
        float radiusFeet=innerRadiusFeet+feetRobotWidth/2;
        float radius=feetToMeters(radiusFeet);

        for(float theta=.5f;theta<=Math.PI*1.5-.1;theta+=.5){
            Translation2d turnOffset=new Translation2d(radius*Math.cos(theta),radius*Math.sin(theta));
            Translation2d turnPosition=turnCenter.plus(turnOffset);
            path.add(turnPosition);
        }
        Translation2d turnExit=turnEntryExitTopLeft.plus(new Translation2d(feetToMeters(feetRobotHeight)/2,-feetToMeters(feetRobotWidth)/2));
        path.add(turnExit);
        
        return path;
    }
    public static void initSmartDashboard(){
        SmartDashboard.getEntry("Max voltage").addListener((EntryNotification e)-> maxVoltage=(float)e.value.getDouble(), EntryListenerFlags.kUpdate|EntryListenerFlags.kNew);
        SmartDashboard.putNumber("Max voltage", maxVoltage);

        SmartDashboard.getEntry("Max Accel").addListener((EntryNotification e)-> maxAccel=(float)e.value.getDouble(), EntryListenerFlags.kUpdate|EntryListenerFlags.kNew);
        SmartDashboard.putNumber("Max Accel", maxAccel);

        SmartDashboard.getEntry("Max Speed").addListener((EntryNotification e)-> maxSpeed=(float)e.value.getDouble(), EntryListenerFlags.kUpdate|EntryListenerFlags.kNew);
        SmartDashboard.putNumber("Max Speed", maxSpeed);

        SmartDashboard.getEntry("Inner Radius Feet").addListener((EntryNotification e)-> innerRadiusFeet=(float)e.value.getDouble(), EntryListenerFlags.kUpdate|EntryListenerFlags.kNew);
        SmartDashboard.putNumber("Inner Radius Feet", innerRadiusFeet);
    }

    public TestPrecisionCommand(DriveTrain robotDrive) {
        this.robotDrive=robotDrive;
    }
    @Override
    public void initialize() {
        robotDrive.resetOdometry(simulationInitialPose);
        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.ksVolts,
                                           Constants.kvVoltSecondsPerMeter,
                                           Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics,
                maxVoltage);
        
        TrajectoryConfig config =
            new TrajectoryConfig(maxSpeed,
                                 maxAccel)
                .setKinematics(Constants.kDriveKinematics)
                .addConstraint(autoVoltageConstraint);
            
        ArrayList<Translation2d> path=generatePath(innerRadiusFeet);
        Pose2d finish=new Pose2d(path.get(path.size()-1).plus(new Translation2d(feetToMeters(2), 0)),new Rotation2d(0));

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            simulationInitialPose,
            path,
            finish,
            config
        ); 
    
    
    
        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
            robotDrive::getPose,
            new RamseteController(Constants.kRamseteB*5, Constants.kRamseteZeta),
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
        command= ramseteCommand.andThen(() -> robotDrive.tankDriveVolts(0, 0));
        command.initialize();
      }
      @Override
      public void execute() {
          command.execute();
      }
      @Override
      public void end(boolean interrupted) {

          command.end(interrupted);
      }
      @Override
      public boolean isFinished() {
          return command.isFinished();
      }
      @Override
      public Pose2d getInitialPose() {
          return simulationInitialPose;
      }
}

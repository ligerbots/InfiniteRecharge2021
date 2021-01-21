package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
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
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;


public class TestPrecisionCommandUturn extends CommandBase implements AutoCommandInterface{
    static Pose2d simulationInitialPose=new Pose2d(Units.feetToMeters(10),Units.feetToMeters(3),new Rotation2d(Math.PI/2));
    static double feetRunway=10;
    static double feetRobotWidth=27.f/12.f;
    static double feetRobotHeight =28.f/12.f;
    static double maxSpeed= Constants.kMaxSpeed;
    static double maxAccel=  Constants.kMaxAcceleration;
    static double maxVoltage=10;
    static double innerRadiusFeet=1;
    DriveTrain robotDrive;
    SequentialCommandGroup command;

    
    static ArrayList<Translation2d> generatePath(double innerRadiusFeet){
        ArrayList<Translation2d> path=new ArrayList<>();
        Translation2d turnEntry=simulationInitialPose.getTranslation().plus(new Translation2d(0,Units.feetToMeters(feetRunway)));
        path.add(turnEntry);

        Translation2d turnEntryExitTopLeft=turnEntry.plus(new Translation2d(-Units.feetToMeters(feetRobotWidth)/2,Units.feetToMeters(feetRobotHeight)/2));
        Translation2d turnCenter=turnEntryExitTopLeft.plus(new Translation2d(-Units.feetToMeters(innerRadiusFeet),Units.feetToMeters(innerRadiusFeet)));
        double radiusFeet=innerRadiusFeet+feetRobotWidth/2;
        double radius=Units.feetToMeters(radiusFeet);

        for(double theta=.5f;theta<=Math.PI-.1;theta+=.5){
            Translation2d turnOffset=new Translation2d(radius*Math.cos(theta),radius*Math.sin(theta));
            Translation2d turnPosition=turnCenter.plus(turnOffset);
            path.add(turnPosition);
        }
        Translation2d turnExit=turnEntry.plus(new Translation2d(-2*radius, 0));
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

        SmartDashboard.getEntry("Feet straightforward").addListener((EntryNotification e)-> feetRunway=(float)e.value.getDouble(), EntryListenerFlags.kUpdate|EntryListenerFlags.kNew);
        SmartDashboard.putNumber("Feet straightforward", feetRunway);
    }

    public TestPrecisionCommandUturn(DriveTrain robotDrive) {
        this.robotDrive=robotDrive;
    }
    @Override
    public void initialize() {
        robotDrive.setPose(simulationInitialPose);
        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.ksVolts,
                                           Constants.kvVoltSecondsPerMeter,
                                           Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics,
                maxVoltage);
        
        TrajectoryConfig config =
            new TrajectoryConfig(maxSpeed, maxAccel)
                .setKinematics(Constants.kDriveKinematics)
                .addConstraint(autoVoltageConstraint);
            
        ArrayList<Translation2d> path=generatePath(innerRadiusFeet);
        Pose2d finish=new Pose2d(path.get(path.size()-1).plus(new Translation2d(0, -Units.feetToMeters(feetRunway))),new Rotation2d(Units.degreesToRadians(-90)));
        
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
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
        command = ramseteCommand.andThen(() -> robotDrive.tankDriveVolts(0, 0));
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

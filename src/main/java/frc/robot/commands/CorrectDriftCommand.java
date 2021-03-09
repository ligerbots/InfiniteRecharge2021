package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class CorrectDriftCommand extends CommandBase {    
    String pathName;
    TrajectoryConfig traj_config;
    double[] emptyArray = new double[0];
    Pose2d traj_start; 
    List<Translation2d> traj_interiorWaypoints; 
    Pose2d traj_end;
    boolean resetPosition;
    double[] makePoints(Pose2d start, List<Translation2d> waypoints, Pose2d end){
        double[] res =new double[3+waypoints.size()*2+3];
        res[0]=start.getX();
        res[1]=start.getY();
        res[2]=start.getRotation().getRadians();
        for(int i=0;i<waypoints.size();i++){
            res[3+2*i] = waypoints.get(i).getX();
            res[3+2*i+1] = waypoints.get(i).getY();
        }
        res[res.length-3] = end.getX();
        res[res.length-2] = end.getY();
        res[res.length-1] = end.getRotation().getRadians();
        return(res);
    }
    Trajectory makeTraj(double[] points){
        if(points.length<6) return null;
        Pose2d start= new Pose2d(points[0],points[1], new Rotation2d(points[2]));
        Pose2d end= new Pose2d(points[points.length-3],points[points.length-2], new Rotation2d(points[points.length-1]));
        ArrayList<Translation2d> waypoints =new ArrayList<>();
        for(int i=3;i<points.length-3;i+=2){
            waypoints.add(new Translation2d(points[i],points[i+1]));
        }
        return(TrajectoryGenerator.generateTrajectory(start, waypoints, end, traj_config));
    }

    DriveTrain driveTrain;
    RamseteCommand ramsete;
    public CorrectDriftCommand(DriveTrain robotDrive, boolean resetPosition, String pathName, Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, TrajectoryConfig config){
        driveTrain=robotDrive;
        this.resetPosition=resetPosition;
        this.pathName = pathName;
        traj_start=start;
        traj_interiorWaypoints = interiorWaypoints;
        traj_end=end;
        traj_config=config;
        
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("correctdrift/run", false);
        SmartDashboard.putString("correctdrift/path_name", pathName);
        SmartDashboard.putNumberArray("correctdrift/original_points", makePoints(traj_start,traj_interiorWaypoints,traj_end));
        ramsete = null;
    }

    @Override
    public void execute() {
        if (ramsete == null){
            if(SmartDashboard.getBoolean("correctdrift/run", false)){
                Trajectory traj = makeTraj(SmartDashboard.getNumberArray("correctdrift/modified_points",emptyArray));
                SmartDashboard.putBoolean("correctdrift/run", false);

                if(traj != null){
                    Pose2d init = traj.getInitialPose();

                    if(resetPosition) driveTrain.setPose(init);
                    
                    ramsete = new RamseteCommand(
                        traj,
                        driveTrain::getPose,
                        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                        new SimpleMotorFeedforward(Constants.ksVolts,
                                                Constants.kvVoltSecondsPerMeter,
                                                Constants.kaVoltSecondsSquaredPerMeter),
                        Constants.kDriveKinematics,
                        driveTrain::getWheelSpeeds,
                        new PIDController(Constants.kPDriveVel, 0, 0),
                        new PIDController(Constants.kPDriveVel, 0, 0),
                        driveTrain::tankDriveVolts,
                        driveTrain
                    );
                    ramsete.initialize();

                }
            }
        }else{

            ramsete.execute();
        }
    }
    @Override
    public void end(boolean interrupted) {
        if(ramsete != null) ramsete.end(interrupted);
    }
    @Override
    public boolean isFinished() {
        if(ramsete == null) return false;
        return ramsete.isFinished();
    }
    
}

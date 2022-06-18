package frc.robot.commands;

import java.io.File;
import java.io.PrintWriter;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;

// Class to write out the Pose along a trajectory
// This can be used outside to plot the curve

public class TrajectoryWriter {
    static final double TIME_INCREMENT = 0.02;

    String m_name;
    File m_file;
    PrintWriter m_csv;

    public TrajectoryWriter(String name) {
        m_name = name;
        String filename = "trajectory_" + name + ".csv";

        m_file = new File(filename);
        try {
            m_csv = new PrintWriter(m_file);
            m_csv.println("IsWaypoint,Time,X,Y,Heading");
        } catch (Exception e) {
            System.err.println("error opening file");
        }
    }
    
    public void WriteTrajectory(Trajectory trajectory) {
        double time = -TIME_INCREMENT;
        double maxTime = trajectory.getTotalTimeSeconds();

        while ( time < maxTime ) {
            time += TIME_INCREMENT;
            if ( time > maxTime ) time = maxTime;
            Pose2d p = trajectory.sample(time).poseMeters;
            WritePose(false, time, p);
        }

        m_csv.flush();
    }

    public void WriteWaypoints(Pose2d initPose, List<Translation2d> waypointList, Pose2d endPose) {
        WritePose(true, 0, initPose);
        for ( Translation2d t : waypointList ) {
            WriteTranslate2d(t);
        }
        WritePose(true, 0, endPose);

        m_csv.flush();
    }

    private void WritePose(boolean isWay, double t, Pose2d p)
    {
        m_csv.println((isWay ? 1 : 0) + "," + t + ","
                + Units.metersToInches(p.getX()) + "," + Units.metersToInches(p.getY()) 
                + "," + p.getRotation().getDegrees());     
    }

    private void WriteTranslate2d(Translation2d t)
    {
        m_csv.println("1,0," + Units.metersToInches(t.getX()) + "," + Units.metersToInches(t.getY()) + ",");     
    }
}
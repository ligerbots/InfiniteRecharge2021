package frc.robot.commands;

import java.io.File;
import java.io.PrintWriter;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;

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
            m_csv.println("Time,X,Y,Heading");
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
            m_csv.println(time + "," + Units.metersToInches(p.getX()) + "," + Units.metersToInches(p.getY()) + "," + p.getRotation().getDegrees());
        }

        m_csv.flush();
    }
}
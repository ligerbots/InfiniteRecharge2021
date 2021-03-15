package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

// Static class for simple methods to plot a trajectory on the Field map

public class TrajectoryPlotter {
    private Field2d m_field2d;

    public TrajectoryPlotter(Field2d field) {
        m_field2d = field;
    }

    public void clear() {
        // Seems to be the only way to clear the lists
        m_field2d.getObject("trajectory").setPoses(Collections.<Pose2d>emptyList());
        m_field2d.getObject("waypoints").setPoses(Collections.<Pose2d>emptyList());
    }

    public void plotTrajectory(Trajectory trajectory) {
        m_field2d.getObject("trajectory")
                .setPoses(trajectory.getStates().stream()
                .map(state -> state.poseMeters).collect(Collectors.toList()));
    }

    public void plotWaypoints(List<Translation2d> waypoints) {
        final Rotation2d rot = Rotation2d.fromDegrees(0);

        List<Pose2d> poses = new ArrayList<Pose2d>();
        for (Translation2d t : waypoints) {
            poses.add(new Pose2d(t, rot));
        }

        m_field2d.getObject("waypoints").setPoses(poses);
    }
}
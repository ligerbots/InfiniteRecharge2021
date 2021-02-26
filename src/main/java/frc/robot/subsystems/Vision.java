package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
// import frc.robot.FieldMap;
import frc.robot.FieldMapHome;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
    public enum VisionMode {
        INTAKE,                 // driver view through Intake camera
        SHOOTER,                // driver view through Shooter camera
        GOALFINDER,
        BALLFINDER,
        HOPPERFINDER,
        GALACTIC_SEARCH_PATH_CHOOSER,
    }

    private static final double[] EMPTY_TARGET_INFO = new double[] {0.0,0.0,0.0,0.0,0.0,0.0,0.0};

    // half of horizontal field of view of the camera (radians)
    private static final double HORIZONTAL_HALF_FOV = Math.toRadians(32.0);

    // maximum reasonable distance where we can find the target (inches)
    private static final double MAX_DISTANCE = 30.0 * 12.0;

    private Relay spike;
    private DriveTrain driveTrain;

    // results array for simulation mode
    // keep a copy around so we don't have to allocate the array every loop
    private double[] targetInfoSim = new double[] {0.0,0.0,0.0,0.0,0.0,0.0,0.0};

    // Note: driveTrain is needed for simulation mode only

    public Vision(DriveTrain driveTrain) {
        spike = new Relay(0);
        this.driveTrain = driveTrain;

        // start the camera in GoalFinder, for auto, but don't turn on the LED
        this.setMode(VisionMode.GOALFINDER, false);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        if (getMode() == VisionMode.GOALFINDER) {
            // Competition field value
            // Translation2d goalPos = FieldMap.goalCenterPoint;

            // Shed value
            Translation2d goalPos = new Translation2d(0.0, FieldMapHome.fieldWidth/2.0);
  
            Translation2d goalDiff = goalPos.minus(driveTrain.getPose().getTranslation());
            double distance = goalDiff.getNorm() / Constants.inchToMetersConversionFactor;

            double angleRobotGoal = Math.atan2(goalDiff.getY(), goalDiff.getX());
            double visionAngle = Math.toRadians(driveTrain.getHeading()) - angleRobotGoal;
            while (visionAngle > Math.PI) visionAngle -= 2.0*Math.PI;
            while (visionAngle < -Math.PI) visionAngle += 2.0*Math.PI;

            if ( distance < MAX_DISTANCE && Math.abs(visionAngle) < HORIZONTAL_HALF_FOV) {
                // target should be visible to the robot
                targetInfoSim[1] = 1;
                targetInfoSim[3] = distance;
                targetInfoSim[4] = visionAngle;
            } else {
                targetInfoSim[1] = 0;
                targetInfoSim[3] = 0;
                targetInfoSim[4] = 0;
            }
        } else if (getMode() == VisionMode.GALACTIC_SEARCH_PATH_CHOOSER) {
            // for now assume always a-red path

            SmartDashboard.putString("vision/galactic_search_path_chooser/result", "a-red");
            targetInfoSim[1] = 1;
        } else {
            targetInfoSim[1] = 1;
            targetInfoSim[3] = 0;
            targetInfoSim[4] = 0;
        }

        // include the heartbeat signal that Vision puts in
        targetInfoSim[0] = Robot.time();

        SmartDashboard.putNumberArray("vision/target_info", targetInfoSim);
    }

    // set vision processing mode using enum, and set LED to match what is needed
    public void setMode(VisionMode mode) {
        setMode(mode, mode == VisionMode.GOALFINDER || mode == VisionMode.HOPPERFINDER);
    }

    // set vision processing mode, and full control of LED
    // should be used from outside only for special cases
    public void setMode(VisionMode mode, boolean led) {
        SmartDashboard.putString("vision/active_mode/selected", mode.name().toLowerCase());
        setLedRing(led);
    }
    
    // what mode is the vision processing running in
    public VisionMode getMode() {
        String mode = SmartDashboard.getString("vision/active_mode/selected", "");
        try {
            return VisionMode.valueOf(mode.toUpperCase());
        } catch(Exception e) {
            // ignore
        }
        return VisionMode.INTAKE;
    }

    public boolean getStatus() {
        double[] visionData = SmartDashboard.getNumberArray("vision/target_info", EMPTY_TARGET_INFO);
        // 1 = success, but it comes as a float, so test with a greater-than
        return visionData[1] > 0.1;
    }
    
    public double getDistance() {
        double[] visionData = SmartDashboard.getNumberArray("vision/target_info", EMPTY_TARGET_INFO);
        return visionData[3];
    }
    
    public double getRobotAngle() {
        double[] visionData = SmartDashboard.getNumberArray("vision/target_info", EMPTY_TARGET_INFO);
        return Math.toDegrees(visionData[4]);
    }


    public enum GalacticSearchChooserResult{
        A_RED,
        A_BLUE,
        B_RED,
        B_BLUE,
        NONE,
    }

    public void resetGalacticSearchChooserResult(){
        SmartDashboard.putString("vision/galactic_search_path_chooser/result", ""); //ensure that old results are not used
    }
    public GalacticSearchChooserResult getGalacticSearchChooserResult(){
        if(!getStatus())return GalacticSearchChooserResult.NONE;

        String result = SmartDashboard.getString("vision/galactic_search_path_chooser/result", "");

        try {
            return GalacticSearchChooserResult.valueOf(result.toUpperCase().replace('-', '_'));
        } catch(Exception e) {
            if(!result.equals("")) System.out.println("unknown result "+result);
        }
        return GalacticSearchChooserResult.NONE;
    }

    public void setLedRing (boolean on) {
        spike.set(on ? Value.kReverse : Value.kOff);
    }
}

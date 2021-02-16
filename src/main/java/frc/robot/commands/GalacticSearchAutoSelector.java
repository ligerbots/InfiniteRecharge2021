/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.GalacticSearchAuto.Path;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.GalacticSearchChooserResult;
import frc.robot.subsystems.Vision.VisionMode;

public class GalacticSearchAutoSelector extends CommandBase implements AutoCommandInterface {
  /**
   * Creates a new GalacticSearchAutoSelector.
   */

  Vision vision;
  DriveTrain robotDrive;
  DriveCommand drivecommand;
  Carousel carousel;
  Intake intake;

  Pose2d initialPose = new Pose2d();
  Vision.GalacticSearchChooserResult result = GalacticSearchChooserResult.NONE;
  HashMap<GalacticSearchChooserResult, GalacticSearchAuto> galacticSearchAutos;
  public GalacticSearchAutoSelector(DriveTrain robotDrive, DriveCommand drivecommand, Carousel carousel, Intake intake, Vision vision) {
    this.vision = vision;
    this.robotDrive = robotDrive;
    this.drivecommand = drivecommand;
    this.carousel = carousel;
    this.intake = intake;

    galacticSearchAutos = new HashMap<>();
    galacticSearchAutos.put(GalacticSearchChooserResult.A_RED, new GalacticSearchAuto(robotDrive, drivecommand, carousel, intake, Path.RedA));
    galacticSearchAutos.put(GalacticSearchChooserResult.A_BLUE, new GalacticSearchAuto(robotDrive, drivecommand, carousel, intake, Path.BlueA));
    galacticSearchAutos.put(GalacticSearchChooserResult.B_RED, new GalacticSearchAuto(robotDrive, drivecommand, carousel, intake, Path.RedB));
    galacticSearchAutos.put(GalacticSearchChooserResult.B_BLUE, new GalacticSearchAuto(robotDrive, drivecommand, carousel, intake, Path.BlueB));
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    result=GalacticSearchChooserResult.NONE;
    vision.resetGalacticSearchChooserResult();
    vision.setMode(VisionMode.GALACTIC_SEARCH_PATH_CHOOSER);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    result = vision.getGalacticSearchChooserResult();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Scheduling " + result);
    GalacticSearchAuto command = galacticSearchAutos.getOrDefault(result, null);
    
    if(command != null){
      robotDrive.setPose(command.getInitialPose());
      command.schedule();
    }
    vision.setMode(VisionMode.INTAKE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return result != GalacticSearchChooserResult.NONE;
  }

  @Override
  public Pose2d getInitialPose() {
    return initialPose; // shouldn't be in this state for long
  }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.GalacticSearchChooserResult;
import frc.robot.subsystems.Vision.VisionMode;

public class GalacticSearchAutoSelector extends CommandBase implements AutoCommandInterface {
  /**
   * Creates a new GalacticSearchAutoSelector.
   */

  Vision vision;
  Vision.GalacticSearchChooserPathHint pathHint;
  Vision.GalacticSearchChooserResult result = GalacticSearchChooserResult.NONE;

  public GalacticSearchAutoSelector(Vision vision, Vision.GalacticSearchChooserPathHint pathHint) {
    this.vision = vision;
    this.pathHint = pathHint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vision.setGalacticSearchChooserPathHint(this.pathHint);
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
    // kinda a hack but sendablechooser doesn't have a method to select a option
    // exposed
    System.out.println("Setting auto to " + result);
    switch (result) {
      case A_RED:
        SmartDashboard.putString("Chosen Auto/selected", "RedAAuto");
        break;
      case A_BLUE:
        SmartDashboard.putString("Chosen Auto/selected", "BlueAAuto");
        break;
      case B_RED:
        SmartDashboard.putString("Chosen Auto/selected", "RedBAuto");
        break;
      case B_BLUE:
        SmartDashboard.putString("Chosen Auto/selected", "BlueBAuto");
        break;
    }
    // no i do not like this either
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return result != GalacticSearchChooserResult.NONE;
  }

  @Override
  public Pose2d getInitialPose() {
    return new Pose2d(); // shouldn't be in this state for long
  }
}
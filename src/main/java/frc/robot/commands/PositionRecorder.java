/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.Date;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class PositionRecorder extends CommandBase {
  /**
   * Creates a new PositionRecorder.
   */
  static final String NETWORKTABLES_NAME="Position Recorder";
  static final String NETWORKTABLES_DIRECTORY_NAME="Position Recorder Directory";
  public boolean isRunning = false;

  DriveTrain drivetrain;
  PrintWriter writer;
  long start;

  String directoryName="position-recordings";

  public PositionRecorder(DriveTrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    initSmartDashboard();
    this.drivetrain = drivetrain;
  }
  public void initSmartDashboard(){
    SmartDashboard.getEntry(NETWORKTABLES_NAME).addListener((EntryNotification e)-> setIsRunning(e.value.getBoolean()), EntryListenerFlags.kUpdate|EntryListenerFlags.kNew);
    SmartDashboard.putBoolean(NETWORKTABLES_NAME, isRunning);

    SmartDashboard.getEntry(NETWORKTABLES_DIRECTORY_NAME).addListener((EntryNotification e)-> directoryName=e.value.getString(), EntryListenerFlags.kUpdate|EntryListenerFlags.kNew);
    SmartDashboard.putString(NETWORKTABLES_DIRECTORY_NAME, directoryName);
  }

  public void setIsRunning(boolean running){
    System.out.println("Set is running: "+running);
    if(running!=isRunning){
      if(running){
        this.schedule();
      }else{
        this.cancel();
      }
    }
    if(SmartDashboard.getBoolean(NETWORKTABLES_NAME, false)!=running){
      SmartDashboard.putBoolean(NETWORKTABLES_NAME, running);
    }
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isRunning = true;
    writer=null;
    System.out.println("Position Recorder Begin");
    try {
      String filename = new SimpleDateFormat("MM-dd_HH_mm_ss").format(new Date())+".csv";

      new File(directoryName).mkdirs();

      File f = new File(directoryName,filename);
      f.createNewFile();
      writer = new PrintWriter(f); // PrintWriter is buffered
      writer.println("Elapsed milliseconds,x,y,rotation");

      start=System.currentTimeMillis();

      System.out.println("Writing to: "+f.getAbsolutePath());

    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(writer==null)return;
    Pose2d currentPosition=drivetrain.getPose();
    writer.println((System.currentTimeMillis()-start)+","+currentPosition.getX()+","+currentPosition.getY()+","+currentPosition.getRotation().getRadians());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Position Recorder END");

    isRunning=false;

    if(writer==null)return;
    writer.close();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
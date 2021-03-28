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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class PositionRecorder extends CommandBase {
  /**
   * Creates a new PositionRecorder.
   */
  public boolean isRunning = false;

  DriveTrain drivetrain;
  PrintWriter writer;
  double start;

  String directoryName="position-recordings";

  NetworkTableEntry directoryNameEntry;
  NetworkTableEntry isRunningEntry;
  
  public PositionRecorder(DriveTrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    initSmartDashboard();
    this.drivetrain = drivetrain;
  }
  public void initSmartDashboard(){
    ShuffleboardTab tab =Shuffleboard.getTab("Position Recorder");
    isRunningEntry=tab.add("Is running", isRunning).withWidget("Toggle Button").getEntry();
    directoryNameEntry=tab.add("Directory Name", directoryName).getEntry();


    isRunningEntry.addListener((EntryNotification e)-> setIsRunning(e.value.getBoolean()), EntryListenerFlags.kUpdate|EntryListenerFlags.kNew);

    directoryNameEntry.addListener((EntryNotification e)-> directoryName=e.value.getString(), EntryListenerFlags.kUpdate|EntryListenerFlags.kNew);
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
    if(isRunningEntry.getBoolean(false)!=running){
      isRunningEntry.setBoolean(running);
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
      writer.println("elapsed seconds,x,y,rotation");

      start=Robot.time();

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
    try{
      writer.println((Robot.time()-start)+","+currentPosition.getX()+","+currentPosition.getY()+","+currentPosition.getRotation().getRadians());
    }catch(Exception e){
      e.printStackTrace();
      writer.close();
      writer=null;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Position Recorder END");

    isRunning=false;

    if(writer!=null){
      writer.close();
      writer=null;
    }
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
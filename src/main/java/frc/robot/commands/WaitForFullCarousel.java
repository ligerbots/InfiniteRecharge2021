// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.Carousel;

public class WaitForFullCarousel extends CommandBase {
    Carousel m_carousel;

    // SmartDashboard wants only 1 tab with a specific name.
    // Easiest solution is to make this static so it is shared between
    // all the instances.
    // That's OK because there is only one SmartDashboard entry anyway.
    static NetworkTableEntry s_ballsLoadedEntry = null;

    public WaitForFullCarousel(Carousel carousel){
        m_carousel = carousel;
        if (s_ballsLoadedEntry == null) initSmartDashboard();
    }

    void initSmartDashboard() {
        // Use a tab with a button. If for a real competition, a controller button would be better.
        ShuffleboardTab tab = Shuffleboard.getTab("Interstellar Accuracy Shooter");
        s_ballsLoadedEntry = tab.add("Balls loaded", false).withWidget("Toggle Button").getEntry();    
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Waiting for carousel or dashboard button");
        // Clear the value in SmartDashboard
        s_ballsLoadedEntry.setBoolean(false);
    }

    @Override
    public void end(boolean interrupted) {
        // Clear the value in SmartDashboard
        s_ballsLoadedEntry.setBoolean(false);
        System.out.println("WaitForFullCarousel ended");
    }

    @Override
    public boolean isFinished() {
        // End if SD value is True 
        return s_ballsLoadedEntry.getBoolean(false)
          || m_carousel.getBallCount() >= Constants.CAROUSEL_MAX_BALLS;
    }
}

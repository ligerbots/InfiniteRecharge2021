// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.Carousel;

public class WaitForFullCarousel extends CommandBase {
    Carousel m_carousel;

    public WaitForFullCarousel(Carousel carousel){
        m_carousel = carousel;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }
    
    @Override
    public void end(boolean interrupted) {
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_carousel.getBallCount() >= Constants.CAROUSEL_MAX_BALLS;
    }
}

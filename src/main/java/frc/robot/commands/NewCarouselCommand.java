package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Carousel;

public class NewCarouselCommand extends CommandBase {
  /**
   * Creates a new CarouselCommand.
   */

  Carousel carousel;

  double lastTimeCheck;
  boolean slotFull;

  final double sensorWaitTime = 0.04; // seconds

  double targetSlot;

  double sensorStartTime;

  //TODO find a better value
  final double earlySlotStopDelta = 0.04;
  
  private static enum State {
    Rotating,  WaitingForBall, Full, WaitingForSensor;

  } 

  State state;

  public NewCarouselCommand(Carousel carousel) {
    this.carousel = carousel;
    addRequirements(carousel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastTimeCheck = Robot.time();
    state = State.WaitingForBall;
    slotFull = carousel.isBallInFront();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //     State: Rotating,  WaitingForBall, Full, WaitForSensor

    if (state == State.Full) {
      return;
    } 
    // start in WaitingForBall
    // if WaitForSensor:
    if (state == State.WaitingForSensor) {
      if (Robot.time() - sensorStartTime >= sensorWaitTime) {
        state = State.WaitingForBall;
      }
    }
    //     wait for PauseTime (for sensor to settle)
    //     goto WaitingForBall
    // if WaitingForBall:
    //     carousel stopped
    
    if (state == State.WaitingForBall) {
      // check for ball, if YES:
      if (carousel.isBallInFront()) {
        carousel.incrementBallCount();
        if (carousel.getBallCount() >= Constants.CAROUSEL_MAX_BALLS) {
          state = State.Full;
        } else {
          targetSlot = Math.round(carousel.getSlot()) + 1.0;
          carousel.spin(Constants.CAROUSEL_INTAKE_SPEED);
          state = State.Rotating;
        }
      }
    }
    
    //         incrementCount of balls
    //         if ball count >= MAX:
    //             goto Full
    //         change to Rotating
    //         remember slot number (or remember target slot)
    if (state == State.Rotating) {
      if (carousel.getSlot() >= targetSlot - earlySlotStopDelta) {
        sensorStartTime = Robot.time();
        carousel.spin(0.0);
        state = State.WaitingForSensor;

      }
    }
    // if Rotating:
    //     check if at next slot:
    //         look at encoder, close to the next whole number (slot). If at next slot:
    //             stop carousel
    //             change to WaitForSensor
    
    // if Full:
    //     stop spinning
    //     do nothing
        
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
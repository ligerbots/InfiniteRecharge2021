package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Carousel;

public class CarouselCommand extends CommandBase {
  /**
   * Creates a new CarouselCommand.
   */

  Carousel carousel;

  double targetSlot;
  double sensorStartTime;
  
  final double sensorWaitTime = 0.04; // seconds
  //TODO find a better value
  // must be the carousel's overshoot ticks divided by the ticks in one fith of a rotation
  final double earlySlotStopDelta = 1830.0/Constants.CAROUSEL_FIFTH_ROTATION_TICKS;
  
  private static enum State {
    // There are 4 possible states: Rotating,  WaitingForBall, Full, WaitForSensor
    Rotating,  WaitingForBall, Full, WaitingForSensor;

  } 

  private State state;

  public CarouselCommand(Carousel carousel) {
    this.carousel = carousel;
    addRequirements(carousel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // check the carousel is on the center of a slot
    double slotError = Math.abs(Math.round(carousel.getSlot()) - carousel.getSlot());
    if (slotError <= 0.1){
      // carousel is aligned, wait for ball
      state = State.WaitingForBall;
    }
    else{
      // carousel not aligned, rotate to the next slot
      moveToNextSlot();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    // Print state value to smart dashboard
    SmartDashboard.putString("Carousel/state", state.toString());

    if (state == State.Full) {
      // if we have a full carousel, we do nothing and wait for a shooting command
      return;
    } 

    if (state == State.WaitingForSensor) {
      // checks if we have waited at least .04 seconds since the carousel stopped
      // this allows the sensor to settle and get an accurate reading when used
      if (Robot.time() - sensorStartTime >= sensorWaitTime) {
        // sets the state to WaitingForBall
        state = State.WaitingForBall;
      }
    }
    
    if (state == State.WaitingForBall) {
      // checks if there is a ball in the front slot
      if (carousel.isBallInFront()) {
        // increases the ball count by 1
        carousel.incrementBallCount();
        // if there are 3 or more balls in the carousel
        if (carousel.getBallCount() >= Constants.CAROUSEL_MAX_BALLS) {
          state = State.Full;
        } 
        else {
          moveToNextSlot();
        }
      }
    }
    
    if (state == State.Rotating) {
      // checks if we have rotated to a position just ahead of the next slot
      // this allows the carousel to coast to a stop and still land at the right spot
      if (carousel.getSlot() >= targetSlot) {
        // remembers the time that we started to stop
        sensorStartTime = Robot.time();
        carousel.spin(0.0);
        state = State.WaitingForSensor;
      }
    }   
  }

  private void moveToNextSlot(){
    // remembers the position of the next slot we are aiming for
    targetSlot = Math.round(carousel.getSlot()) + 1.0 - earlySlotStopDelta;
    carousel.spin(Constants.CAROUSEL_INTAKE_SPEED);
    state = State.Rotating;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Since isFinshed always returns false, the only way we get here is if we're interrupted.
    // Need to stop the carousel because if it's spinning when we're interrupted, the interrupting
    // command might not expect it to be rotating.
    carousel.spin(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
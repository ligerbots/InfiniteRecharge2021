package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Carousel;

public class CarouselCommand extends CommandBase {
  /**
   * Creates a new CarouselCommand.
   */

  Carousel carousel;

  double lastTimeCheck;
  boolean slotFull;

  final double pauseTime = 0.04; // seconds

  public CarouselCommand(Carousel carousel) {
    this.carousel = carousel;
    addRequirements(carousel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastTimeCheck = Robot.time();
    
    slotFull = carousel.isBallInFront();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //quits if there are 3 or more balls in the carousel (for infinite recharge at home)
    if (carousel.getBallCount() >= 3) {
      return;
    }

    carousel.currentCheckpoint = carousel.getSlot();

    // This is what we do if we aren't going backwards
    if (carousel.currentCheckpoint > carousel.lastCheckpoint) { // if we have indexed to the next slot...
      carousel.lastCheckpoint = carousel.currentCheckpoint;
      lastTimeCheck = Robot.time(); // Start the timer for pausing at a slot
    }
    
    if (Robot.time() - lastTimeCheck > pauseTime && slotFull) {
      // This block executes if we aren't pausing, the slot isn't open, and we haven't already gone around 5 times
      carousel.spin(Constants.CAROUSEL_INTAKE_SPEED); // Spin the carousel
    }
    else { // This block runs if 
      slotFull = carousel.isBallInFront();
      if (slotFull) { // This block runs if there is not ball up front
        carousel.incrementBallCount();
      }
      
      carousel.spin(0); // We aren't supposed to be spinning here
    }
    // This is decently readable go backwards code that runs on a timer 
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
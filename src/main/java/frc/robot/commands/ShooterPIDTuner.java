package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;
public class ShooterPIDTuner {
    private Shooter shooter;
    private ShooterCommand shooterCommand;
    public ShooterPIDTuner(Shooter shooter, ShooterCommand shooterCommand){
        this.shooter = shooter;
        this.shooterCommand = shooterCommand;
    }

    public void SpinUpTune(){
        shooter.calibratePID(0.000145, 1e-8, 0, 6.6774 * 0.00001);
    }

    public void HoldTune(){
        setPID();
    }

    public void getPID(){
        shooterCommand.p = SmartDashboard.getNumber("shooter/P", 0.000145);
        shooterCommand.i = SmartDashboard.getNumber("shooter/I",1e-8);
        shooterCommand.d = SmartDashboard.getNumber("shooter/D", 0);
        shooterCommand.f = SmartDashboard.getNumber("shooter/F", 6.6774 * 0.00001);
    }

    public void setPID(){
        shooter.calibratePID(shooterCommand.p, shooterCommand.i, shooterCommand.d, shooterCommand.f);
    }
}

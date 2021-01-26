package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;
public class ShooterPIDTuner {
    private double p,i,d,f;
    private Shooter shooter;
    public ShooterPIDTuner(Shooter shooter){
        this.shooter = shooter;

    }

    public void spinUpTune(){
        shooter.calibratePID(0.000145, 1e-8, 0, 6.6774 * 1e-5);
    }

    public void HoldTune(){
        getPIDFromDashBoard();
        setPID();
    }

    public void getPIDFromDashBoard(){
        p = SmartDashboard.getNumber("shooter/P", 0.000145);
        i = SmartDashboard.getNumber("shooter/I",1e-8);
        d = SmartDashboard.getNumber("shooter/D", 0);
        f = SmartDashboard.getNumber("shooter/F", 6.6774 * 1e-5);
    }

    public void setPID(){
        shooter.calibratePID(p, i, d, f);
    }
}

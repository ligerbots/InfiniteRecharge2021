package frc.robot.commands;
import frc.robot.subsystems.Shooter;
import com.revrobotics.CANPIDController;
public class ShooterPIDTuner {
    private double P,I,D,F;
    private Shooter shooter;
    CANPIDController pidController;
    public ShooterPIDTuner(Shooter shooter, double initialP, double initialI, double initialD, double initialF, CANPIDController pidController){
        this.shooter = shooter;
        P = initialP; // Setting the inital values for PID
        I = initialI;
        D = initialD;
        F = initialF;
        this.pidController = pidController;
    }

    public void SpinUpTune(){
        shooter.calibratePID(P, I, D, F);
    }

    public void HoldTune(){
        if (!checkPID()){
            //shooter.calibrate();  Need some clarification on what PID values to set if they change
        }
    }

    public boolean checkPID(){
        return (P == pidController.getP() && I == pidController.getI() && D == pidController.getD());
    }


    public void setPID(double p, double i, double d, double f){
        P = p;
        I = i;
        D = d;
        F = f;
    }
}

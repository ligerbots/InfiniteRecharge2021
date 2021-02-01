package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

//Class for the Infinate Recharge At Home challanges Field Map
public class FieldMapHome {

    public static double fieldLength = 360.0;
    public static double fieldWidth = 180.0;

    public FieldMapHome(){

    }
    //When given a grid point for example 'A, 1', returns a translation 2d wth the orresponding coordinates
    public static Translation2d gridPoint(char row, int col){
        
        double y;
        double x;

        //uses row to find the y coordinate
        switch(row){
            case 'A': y = 150.0;
            break;
            case 'B': y = 120.0;
            break;
            case 'C': y = 90.0;
            break;
            case 'D': y = 60.0;
            break;
            case 'E': y = 30.0;
            break;
            default: y = 0.0;
            break;
        }
        //calculates the x coordinate from the col
        x = col * 30;

        return new Translation2d(Units.inchesToMeters(x), Units.inchesToMeters(y));
    }
    // This returns a x and y value based on gridpoints but with x and y offsets.
    public static Translation2d gridPoint(char row, int col, double x, double y){
        Translation2d translation = gridPoint(row,col);
        return new Translation2d(translation.getX() + Units.inchesToMeters(x), translation.getY() + Units.inchesToMeters(y));
    }
}

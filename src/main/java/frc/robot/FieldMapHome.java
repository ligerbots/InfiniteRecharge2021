package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;

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
        //uses col to find the x coordinate
        switch(col){
            case 1: x = 30.0;
            break;
            case 2: x = 60.0;
            break;
            case 3: x = 90.0;
            break;
            case 4: x = 120.0;
            break;
            case 5: x = 150.0;
            break;
            case 6: x = 180.0;
            break;
            case 7: x = 210.0;
            break;
            case 8: x = 240.0;
            break;
            case 9: x = 270.0;
            break;
            case 10: x = 300.0;
            break;
            case 11: x = 330.0;
            break;
            default: x = 0.0;
            break;
        }

        return new Translation2d(x, y);
    }
}

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class FieldMapSkills {
    static HashMap<String,Translation2d> fieldMapPointsByName=new HashMap<>();
    static {
        for(int x=0;x<9;x++){
            for(int y=0;y<5;y++){
                String vname=Character.toString((char) ('E'-y));
                String hname=Integer.toString(x+1);
                double rx=x+1;
                double ry=y+1;
                fieldMapPointsByName.put(vname+hname,new Translation2d(rx,ry));
            }
        }
    }
}

package org.firstinspires.ftc.teamcode.functions;

import com.qualcomm.robotcore.util.Range;

public class ArmRamp {
    public static double rampPos = Constants.ARM_DOWN_POS;
    public static double Ramp(double firstPos, double secondPos, boolean selectPos)
    {
        double rampRate = 0.015;
        if(secondPos>firstPos){
            //Slows down arm servo so it does not break itself
            if(selectPos){
                rampPos += rampRate;
            }
            else{
                rampPos -= rampRate;
            }
            rampPos = Range.clip(rampPos,firstPos,secondPos);
        }
        else{
            //Slows down arm servo so it does not break itself
            if(selectPos){
                rampPos -= rampRate;
            }
            else{
                rampPos += rampRate;
            }
            rampPos = Range.clip(rampPos,secondPos,firstPos);
        }
        return rampPos;
    }
}

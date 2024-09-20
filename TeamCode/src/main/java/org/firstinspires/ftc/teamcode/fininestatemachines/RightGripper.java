package org.firstinspires.ftc.teamcode.fininestatemachines;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.functions.Constants;

public class RightGripper {

    public Servo rightGripper;

    public void OpenGripper(){
        rightGripper.setPosition(Constants.GRIPPER_LEFT_OPEN_POSITION);
    }

    public void CloseGripper(){
        rightGripper.setPosition(Constants.GRIPPER_LEFT_CLOSE_POSITION);
    }

    public void RightGripperMaster(boolean openButton, boolean closeButton)
    {
        if(openButton)
            OpenGripper();
        else if(closeButton)
            CloseGripper();
    }
}

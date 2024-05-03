package org.firstinspires.ftc.teamcode.fininestatemachines;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.functions.Constants;

public class LeftGripper {

    public Servo leftGripper;

    public void OpenGripper(){
        leftGripper.setPosition(Constants.GRIPPER_LEFT_OPEN_POSITION);
    }

    public void CloseGripper(){
        leftGripper.setPosition(Constants.GRIPPER_LEFT_CLOSE_POSITION);
    }

    public void LeftGripperMaster(boolean openButton, boolean closeButton)
    {
        if(openButton)
            OpenGripper();
        else if(closeButton)
            CloseGripper();
    }
}

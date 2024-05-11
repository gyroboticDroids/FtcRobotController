package org.firstinspires.ftc.teamcode.fininestatemachines;



public class FieldOrierentedDrive {
    // Variables used for auto turn
    double turnError = 0;
    double turnSpeed = 0;
    double turnSetpoint = 0;
    double turnFeedback = 0;
    double turnOffset = 0;


    public void Direction()
    {
        //Setting auto-turn target position
        if(gamepad1.x){
            turnSetpoint = 90;
        }
        else if(gamepad1.y){
            turnSetpoint = 0;
        }
        else if(gamepad1.b){
            turnSetpoint = 270;
        }
        else if(gamepad1.a){
            turnSetpoint = 180;
        }
        else{
            turnSetpoint += turnOffset;
        }
        //Keeps auto turn between 0 to 360 degrees
        if(turnSetpoint > 360){
            turnSetpoint = turnSetpoint - 360;
        }
        else if(turnSetpoint < 0){
            turnSetpoint = turnSetpoint + 360;
        }
    }
    public void GyroTurn()
    {
        //Calculating auto turn motor powers
        turnError = turnSetpoint - Math.toDegrees(turnFeedback);
        if(turnError > 180){
            turnError = turnError - 360;
        }
        else if(turnError < -180){
            turnError = turnError + 360;
        }

        // Rotation sensitivity
        turnSpeed = 0.04 * turnError;
        // Keeps turning speed in a range
        turnSpeed = Math.min(Math.max(turnSpeed, -0.4), 0.4);
    }
}

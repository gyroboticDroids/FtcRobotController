package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Constants {
    //Constants we use often in all our programs
    public static double GRIPPER_LEFT_OPEN_POSITION = 0;
    public static double GRIPPER_LEFT_CLOSE_POSITION = 0.5;
    public static double GRIPPER_RIGHT_OPEN_POSITION = 0.5;
    public static double GRIPPER_RIGHT_CLOSE_POSITION = 0;
    public static double DRONE_START_POSITION = 0.696;
    public static double DRONE_RELEASE_POSITION = 0.248;

    public static int SLIDE_LOW_POS = 950;
    public static int SLIDE_MEDIUM_POS = 2000;
    public static int SLIDE_HIGH_POS = 3050;

    public static double ARM_UP_POS = 0.744;
    public static double ARM_DOWN_POS = 0.501;
    public static Pose2d autoEndPose = new Pose2d(0, 0, Math.toRadians(0));
    public static boolean blueAuto = false;

}

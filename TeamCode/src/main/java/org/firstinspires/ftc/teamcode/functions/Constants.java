package org.firstinspires.ftc.teamcode.functions;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Constants {
    //Constants we use often in all our programs
    public static double GRIPPER_LEFT_OPEN_POSITION = 0.294;
    public static double GRIPPER_LEFT_CLOSE_POSITION = 0.585;
    public static double GRIPPER_RIGHT_OPEN_POSITION = 0.746;
    public static double GRIPPER_RIGHT_CLOSE_POSITION = 0.455;
    public static double DRONE_START_POSITION = 0.4375;
    public static double DRONE_RELEASE_POSITION = 0.89662;

    public static int SLIDE_LOW_POS = 1100;
    public static int SLIDE_MEDIUM_POS = 2100;
    public static int SLIDE_HIGH_POS = 3050;
    public static int SLIDE_STACK_POS = 305;
    public static int PIXEL_HEIGHT = 332;
    public static int BOARD_BASE_HEIGHT = 225;

    public static double ARM_UP_POS = 0.16;
    public static double ARM_DOWN_POS = 0.51;
    public static Pose2d autoEndPose = new Pose2d(0, 0, Math.toRadians(0));
    public static boolean blueAuto = false;
}

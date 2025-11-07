package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class BotConstants {
    public static final Pose BLUE_GOAL_POSE = new Pose(17, 128);

    public static final double TURRET_ANGLE_SCALAR = 1.10;
    public static final Pose TURRET_AIMING_POSE = new Pose(16, 130);

    public static final double  VELOCITY_COMPENSATION_MULTIPLIER = (double) 2300 /2100;

    public static double M_S_TO_TICKS = 275; // Conversion factor for ball velocity in meters/second to launcher encoder ticks // TODO: needs to be tuned


}

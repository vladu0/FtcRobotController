package org.firstinspires.ftc.teamcode.axis28128;

import com.pedropathing.geometry.Pose;

/**
 * Carries the robot's pose across OpModes (auto -> teleop).
 * Auto writes follower.getPose() into currentPose every loop;
 * teleop reads it in init() as its starting pose.
 */
public class PoseStorage {
    // Default used when teleop runs without an auto first (practice).
    public static Pose currentPose = new Pose(80, 8, 0);
}

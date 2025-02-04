package org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Webcam {

    public static int
            gain = 150,
            exposureMS = 6,
            RESOLUTION_WIDTH = 1920,
            RESOLUTION_HEIGHT = 1080;
    public static boolean
            autoExposure = false;
}
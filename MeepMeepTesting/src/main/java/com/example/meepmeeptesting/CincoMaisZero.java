package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class CincoMaisZero {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(400);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .setDimensions(16.9291, 16.9291)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(8.5, -61, Math.toRadians(-90)))
                        //todo: colocar primeiro specimen
                        .setTangent(90)
                        .strafeTo(new Vector2d(0, -25))
                        //todo: Go to empurrar sample 1
                        //.setTangent(Math.toRadians(90))
                        //.splineToConstantHeading(new Vector2d(34.5, -30),Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(39, -40, Math.toRadians(55)),Math.toRadians(10))



                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

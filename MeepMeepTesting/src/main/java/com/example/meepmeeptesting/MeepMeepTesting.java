package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(400);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 13)
                .setDimensions(16.9291, 16.9291)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(8.5, -61, Math.toRadians(90)))
                        /*.setTangent(Math.toRadians(135))
                        .splineToConstantHeading(new Vector2d(0, -25), Math.toRadians(90))

                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(34, -30, Math.toRadians(-90)), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(34, -5, Math.toRadians(-90)), Math.toRadians(90))*/
                        //todo empurrar sample 1
                        .setTangent(Math.toRadians(45))
                        //strafeTo(new Vector2d(30,-30))
                        //.strafeTo(new Vector2d(44,-5))
                        .splineToLinearHeading(new Pose2d(35, -30, Math.toRadians(90)), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(44, -5, Math.toRadians(90)), Math.toRadians(0))

                        .strafeTo(new Vector2d(44,-55))

                        //todo empurrar sample 2

                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(56, -10, Math.toRadians(90)), Math.toRadians(0))

                        .strafeTo(new Vector2d(56,-55))



                        /*//todo empurrar sample 3
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(62, -10, Math.toRadians(90)), Math.toRadians(0))

                        .strafeTo(new Vector2d(62,-60))*/

                        //todo intake sample 1
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(56, -50, Math.toRadians(90)), Math.toRadians(90))
                        .setTangent(-90)
                        .splineToLinearHeading(new Pose2d(56, -55, Math.toRadians(90)), Math.toRadians(-90))

                        //todo move deposito sample 1
                        .setTangent(Math.toRadians(135))
                        //.splineToLinearHeading(new Pose2d(0, -30, Math.toRadians(-90)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(0, -30, Math.toRadians(-90)), Math.toRadians(90))

                        //todo intake
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(38, -60, Math.toRadians(-90)), Math.toRadians(-90))

                        //todo deposito
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(new Pose2d(0, -30, Math.toRadians(-90)), Math.toRadians(90))

                        //todo intake
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(38, -60, Math.toRadians(-90)), Math.toRadians(-90))

                        //todo deposito
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(new Pose2d(0, -30, Math.toRadians(-90)), Math.toRadians(90))

                        //todo intake
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(38, -60, Math.toRadians(-90)), Math.toRadians(-90))

                        //todo deposito
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(new Pose2d(0, -30, Math.toRadians(-90)), Math.toRadians(90))

                        //todo move final
                        .strafeTo(new Vector2d(0,-40))




                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
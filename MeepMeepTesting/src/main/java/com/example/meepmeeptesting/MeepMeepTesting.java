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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(8.5, -61, Math.toRadians(-90)))
                        .setTangent(Math.toRadians(135))
                        .splineToConstantHeading(new Vector2d(0, -25), Math.toRadians(90))

                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(34, -30, Math.toRadians(-90)), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(34, -5, Math.toRadians(-90)), Math.toRadians(90))

                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(44, -5, Math.toRadians(-90)), Math.toRadians(-90))

                        //.setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(44, -52, Math.toRadians(-85)), Math.toRadians(-90))

                        //todo empurrar sample 2
                        //.setReversed(true)
                        //.splineToLinearHeading(new Pose2d(44, -10, Math.toRadians(-90)), Math.toRadians(90))
                        //.lineToY(-10)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(56, -10, Math.toRadians(-90)), Math.toRadians(-90))

                        //.setTangent(Math.toRadians(-90))
                        //.splineToLinearHeading(new Pose2d(56, -52, Math.toRadians(-85)), Math.toRadians(-90))
                        //.lineToY(-52)
                        //todo empurrar sample 3
                        //.lineToY(-10)
                        //.splineToLinearHeading(new Pose2d(54, -10, Math.toRadians(-90)), Math.toRadians(90))

                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(66, -10, Math.toRadians(-90)), Math.toRadians(-90))

                        //.setTangent(Math.toRadians(-90))
                        //.splineToLinearHeading(new Pose2d(63, -52, Math.toRadians(-85)), Math.toRadians(-85))
                        //.lineToY(-52)
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(60, -45, Math.toRadians(-85)), Math.toRadians(-85))
                        .waitSeconds(0.2)
                        //.lineToY(-63)

                        /*//todo move deposito
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(0, -20, Math.toRadians(-90)), Math.toRadians(90))
                        //todo empurrar o sample 1
                        .setTangent(Math.toRadians(-90))
                       
                        .splineToLinearHeading(new Pose2d(38, -35, Math.toRadians(-90)), Math.toRadians(90))

                        .splineToLinearHeading(new Pose2d(38, -5, Math.toRadians(-90)), Math.toRadians(90))

                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(48, -5, Math.toRadians(-90)), Math.toRadians(-90))

                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(48, -50, Math.toRadians(-90)), Math.toRadians(-90))


                        //todo empurrar sample 2
                        .setTangent(Math.toRadians(100))
                        .splineToLinearHeading(new Pose2d(48, -10, Math.toRadians(-90)), Math.toRadians(90))

                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(56, -10, Math.toRadians(-90)), Math.toRadians(-90))

                        //.setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(56, -50, Math.toRadians(-90)), Math.toRadians(-90))
                        //todo empurrar sample 3
                        .setTangent(Math.toRadians(100))
                        .splineToLinearHeading(new Pose2d(52, -10, Math.toRadians(-90)), Math.toRadians(90))

                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(62, -10, Math.toRadians(-90)), Math.toRadians(-90))

                        //.setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(62, -50, Math.toRadians(-90)), Math.toRadians(-90))
                        //intake
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(38, -60, Math.toRadians(-90)), Math.toRadians(-90))
                        /*
                        .setTangent(-135)
                        .splineToLinearHeading(new Pose2d(58, -60, Math.toRadians(-90)), Math.toRadians(-90))

                        //todo deposito
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(new Pose2d(0, -20, Math.toRadians(-90)), Math.toRadians(90))

                        //todo intake
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(38, -60, Math.toRadians(-90)), Math.toRadians(-90))

                        //todo deposito
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(new Pose2d(0, -20, Math.toRadians(-90)), Math.toRadians(90))

                        //todo intake
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(38, -60, Math.toRadians(-90)), Math.toRadians(-90))

                        //todo deposito
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(new Pose2d(0, -20, Math.toRadians(-90)), Math.toRadians(90))



                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(38, -50, Math.toRadians(-35)), Math.toRadians(-45))

                        */




                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
package org.firstinspires.ftc.teamcode.opmode.v5_opModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;


@Config
@Autonomous(name = "AutonomoSpecimen", group = "Autonomous")
public class AutoSpecimen extends LinearOpMode {
    V5 robot;

    @Override
    public void runOpMode()  {

        Pose2d initialPose = new Pose2d(8.5, -61, Math.toRadians(-90));

        robot = new V5(hardwareMap,telemetry);
        robot.md.pose = initialPose;
        Actions.runBlocking(
                new SequentialAction(
                        robot.outtakeIntakeSuperior.garraSuperior.fecharGarra(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        robot.outtakeIntakeSuperior.braco.goToInital(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        robot.outtakeIntakeSuperior.garraSuperior.goToOuttakeCHAMBER(),
                        robot.intakeInferior.horizontalInferior.goToRetracted()

                )
        );
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        deposit(),
                        robot.outtakeIntakeSuperior.garraSuperior.abrirGarra(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(0.5).build(),
                        pushSamples()



                )

        );

    }
    public Action Movecomplept(){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        //todo move deposito
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(-90)), Math.toRadians(90))
                        //chegar no sample 2 e empurrar
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(38, -35, Math.toRadians(90)), Math.toRadians(90))

                        .splineToLinearHeading(new Pose2d(38, -5, Math.toRadians(90)), Math.toRadians(90))
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(48, -5, Math.toRadians(90)), Math.toRadians(-90))
                        .setTangent(Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .splineToLinearHeading(new Pose2d(48, -50, Math.toRadians(90)), Math.toRadians(-90))

                        .splineToLinearHeading(new Pose2d(48, -10, Math.toRadians(90)), Math.toRadians(90))
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(56, -10, Math.toRadians(90)), Math.toRadians(-90))
                        .setTangent(Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .splineToLinearHeading(new Pose2d(56, -50, Math.toRadians(90)), Math.toRadians(-90))

                        .splineToLinearHeading(new Pose2d(52, -10, Math.toRadians(90)), Math.toRadians(90))
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(62, -10, Math.toRadians(90)), Math.toRadians(-90))
                        .setTangent(Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .splineToLinearHeading(new Pose2d(62, -50, Math.toRadians(90)), Math.toRadians(-90))
                        .setTangent(-135)
                        .splineToLinearHeading(new Pose2d(58, -60, Math.toRadians(-90)), Math.toRadians(-90))

                        //todo deposito
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(-90)), Math.toRadians(90))

                        //todo intake
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(38, -60, Math.toRadians(-90)), Math.toRadians(-90))

                        //todo deposito
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(-90)), Math.toRadians(90))

                        //todo intake
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(38, -60, Math.toRadians(-90)), Math.toRadians(-90))

                        //todo deposito
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(-90)), Math.toRadians(90))

                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(38, -50, Math.toRadians(-35)), Math.toRadians(-45))
                        .build()
        );
    }


    public Action goToDeposit(){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        //todo: colocar primeiro specimen
                        .setTangent(Math.toRadians(135))
                        .splineToConstantHeading(new Vector2d(0, -25), Math.toRadians(90))
                        //todo: Go to empurrar sample 1

                        .build()
        );
    }
    public Action pushSamples() {
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(38, -35, Math.toRadians(-90)), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(38, -5, Math.toRadians(-90)), Math.toRadians(90))

                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(44, -5, Math.toRadians(-90)), Math.toRadians(-90))

                        //.setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(44, -47, Math.toRadians(-90)), Math.toRadians(-90))

                        //todo empurrar sample 2
                        //.setReversed(true)
                        .splineToLinearHeading(new Pose2d(44, -10, Math.toRadians(-90)), Math.toRadians(90))

                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(56, -10, Math.toRadians(-90)), Math.toRadians(-90))

                        //.setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(56, -47, Math.toRadians(-90)), Math.toRadians(-90))

                        //todo empurrar sample 3

                        .splineToLinearHeading(new Pose2d(52, -10, Math.toRadians(-90)), Math.toRadians(90))

                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(62, -10, Math.toRadians(-90)), Math.toRadians(-90))

                        //.setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(62, -50, Math.toRadians(-90)), Math.toRadians(-90))

                        .build()

        );
    }



    public Action firstIntake(){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(46, 48, Math.toRadians(-45)), Math.toRadians(180))
                        .build()
        );
    }


    public Action goToPark() {
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .setTangent(Math.toRadians(-135))
                        .splineToLinearHeading(new Pose2d(30, 10, Math.toRadians(180)), Math.toRadians(-135))
                        .build()
        );

    }

    public Action deposit() {
        return new SequentialAction(
                new ParallelAction(
                        robot.outtakeIntakeSuperior.braco.goToOuttakeCHAMBER(),
                        robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(900),
                        robot.outtakeIntakeSuperior.garraSuperior.goToOuttakeCHAMBER(),
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.5).build(),
                                goToDeposit()
                        )
                )

        );

    }
    public Action intake(){
        return new SequentialAction(

                new ParallelAction(
                        new SequentialAction(

                        )

                )
        );
    }



    public Action sample1() {
        return new SequentialAction(
                new ParallelAction(
                        deposit(),
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                                goToDeposit()
                        )

                ),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.6).build(),
                robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()
        );
    }
    public Action sample2() {
        return new SequentialAction(

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.2).build(),
                intake(),
                deposit()
        );
    }
    public Action sample3() {
        return new SequentialAction(

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.3).build(),
                intake(),
                deposit()
        );
    }
    public Action sample4(){
        return new SequentialAction(

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.3).build(),
                intake(),
                deposit()

        );
    }

    public Action irParaCasa(){
        return new SequentialAction(
                new ParallelAction(
                        //mover lineares para estacionar

                )


        );
    }

}
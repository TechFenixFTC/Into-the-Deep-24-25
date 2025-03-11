package org.firstinspires.ftc.teamcode.opmode.v5_opModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;

@Deprecated
@Disabled
@Config
@Autonomous(name = "AutonomoSpecimen8+0", group = "Autonomous")
public class AutoSpecimen5mais0 extends LinearOpMode {
    V5 robot;
    Action push;
    public static double sample1y = -24, sample2x = 35,sample2y = -43,sample2h = 57;
    public static double  sample3x = 30,sample3y = -39,sample3h = 0;

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
                        robot.outtakeIntakeSuperior.garraSuperior.goToInitialSpecimen(),
                        new InstantAction(() -> {robot.intakeInferior.intakeSuccao.angulacao.setPosition(0.250);})
                        //robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()
                        //robot.intakeInferior.horizontalInferior.goToRetracted()

                )
        );


        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        sample1(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(0.6).build(),
                        //positionIntake(),
                        sample2()

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
        return robot.md.actionBuilder(robot.md.pose)
                        //todo: colocar primeiro specimen
                        .setTangent(Math.toRadians(90))
                        .strafeTo(new Vector2d(0, sample1y))
                        //todo: Go to empurrar sample 1
                        .build();

    }
    public Action goToCupidor() {
        return robot.md.actionBuilder(new Pose2d(sample2x, sample2y, Math.toRadians(sample2h)))
                //.setTangent(Math.toRadians(-45))
                //.splineToLinearHeading(new Pose2d(sample3x, sample3y, Math.toRadians(sample3h)), Math.toRadians(-45))
                .turnTo(Math.toRadians(sample3h))
                .build();
    }

    public Action intake(){
        return new SequentialAction(
                    new ParallelAction(
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.3).build(),
                                positionIntake()
                        ),
                            robot.md.actionBuilder(robot.md.pose)
                                    //.strafeTo(new Vector2d(38 ,-35))
                                    .setTangent(Math.toRadians(-45))
                                    .splineToLinearHeading(new Pose2d(51, -55, Math.toRadians(-90)), Math.toRadians(-90))
                                    .waitSeconds(0.5)
                                    .lineToY(-63)
                                    .build()
                    ),
                collect()

        );
    }

    public Action cuspidor(){
        return new SequentialAction(
                goToCupidor(),
                robot.intakeInferior.intakeSuccao.GotoIntakeSpecimen(),
                new InstantAction(()-> {
                    robot.intakeInferior.intakeSuccao.sugador.setPower(-0.8);
                }),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build()


        );
    }

    public  Action positionIntake() {
        return new ParallelAction(
                                robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()                                                                                                                                                                                                           ,
                                robot.outtakeIntakeSuperior.braco.goToIntakeCHAMBER(),
                                robot.outtakeIntakeSuperior.garraSuperior.goToIntakeSpecimen(),
                                robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(0)


        );
    }
    public Action collect() {
        return new SequentialAction(
                robot.outtakeIntakeSuperior.garraSuperior.fecharGarra(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.8).build()
        );
    }
    public Action collect2() {
        return new SequentialAction(
                //new InstantAction(()->{robot.outtakeIntakeSuperior.garraSuperior.servoAberturaDaGarra.setPosition(2);}),
                robot.md.actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.8).build()
                //robot.md.actionBuilder(robot.md.pose).waitSeconds(0.2).build()
        );
    }

    public Action sample1() {
        return new SequentialAction(
                new ParallelAction(
                        robot.outtakeIntakeSuperior.braco.goToOuttakeCHAMBER(),
                        robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(900),
                        robot.outtakeIntakeSuperior.garraSuperior.goToOuttakeSpecimen(),
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                                goToDeposit()
                        )
                ),

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.25).build(),
                robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()
        );
    }
    public Action sample2() {
        return new SequentialAction(

                robot.md.actionBuilder(new Pose2d(-1.1, sample1y, -90)).waitSeconds(0.2)
                .splineToLinearHeading(new Pose2d(sample2x, sample2y, Math.toRadians(sample2h)), Math.toRadians(30))
                .build(),
                robot.intakeInferior.intakeSuccao.GotoReadyToIntakeSpecimen(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                robot.intakeInferior.linearHorizontalMotor.goToExtended(),
                robot.intakeInferior.intakeSuccao.GotoIntakeSpecimen(),

                new InstantAction(()-> {
                    robot.intakeInferior.intakeSuccao.sugador.setPower(0.8);
                }),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                robot.intakeInferior.intakeSuccao.GotoReadyToIntakeSpecimen(),
                cuspidor()

                //robot.intakeInferior.linearHorizontalMotor.goToRetracted()


        );
    }


}
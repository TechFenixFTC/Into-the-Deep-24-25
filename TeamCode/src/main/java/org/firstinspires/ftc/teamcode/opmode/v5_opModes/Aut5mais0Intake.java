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
@Autonomous(name = "AutonomoSpecimen5+0", group = "Autonomous")
public class Aut5mais0Intake extends LinearOpMode {
    V5 robot;
    Action push;
    public static double sample1y = -25,
            sample2Ax = 37, sample2Ay = -42, sample2Ah = 85,
            sample2Bh = -55,
            sample3Ax = 39, sample3Ay = -40, sample3Ah = 55,
            sample3Bh = -35,
            sample4Ax = 50, sample4Ay = -23, sample4Ah = 0,
            sample4Bx = 45, sample4By = -42, sample4Bh = -35
            ;


    ;


    public static double[] sample4 ={10,10,10};


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
                        outtakeSample1(),
                        collectSample2()
                        //collectSample3()
                        //collectSample4()


                )
        );

    }
    public Action goToDeposit(){
        return robot.md.actionBuilder(robot.md.pose)
                //todo: colocar primeiro specimen
                //.setTangent(Math.toRadians(90))
                .strafeTo(new Vector2d(0, sample1y))
                //todo: Go to empurrar sample 1
                .build();

    }
    public Action goToEjectSample(double posex, double posey, double poseh, double sampleh) {
        return robot.md.actionBuilder(new Pose2d(posex, posey, Math.toRadians(poseh)))
                .turnTo(Math.toRadians(sampleh))
                .build();
    }
    public Action goToEjetingSample3(double posex, double posey, double poseh) {
        return robot.md.actionBuilder(new Pose2d(posex, posey, Math.toRadians(poseh)))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(sample4Bx, sample4By, Math.toRadians(sample4Bh)), Math.toRadians(-90))
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
                robot.intakeInferior.intakeSuccao.GotoIntakeSpecimen(),
                new InstantAction(()-> {
                    robot.intakeInferior.intakeSuccao.sugador.setPower(-0.8);
                }),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                new InstantAction(()-> {
                    robot.intakeInferior.intakeSuccao.sugador.setPower(0.8);
                }),
                robot.intakeInferior.intakeSuccao.GotoReadyToIntakeSpecimen()


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

    public Action outtakeSample1() {
        return new SequentialAction(
                new ParallelAction(
                        robot.outtakeIntakeSuperior.braco.goToOuttakeCHAMBER(),
                        robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(870),
                        robot.outtakeIntakeSuperior.garraSuperior.goToOuttakeSpecimen(),
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                                goToDeposit(),
                                robot.intakeInferior.intakeSuccao.GotoReadyToIntakeSpecimen()
                        )
                ),

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.25).build(),
                robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()
        );
    }

    public Action collectSample2() {//todo okey
        return new SequentialAction(
                new ParallelAction(
                        robot.md.actionBuilder(new Pose2d(0, sample1y, -90))
                                .setTangent(Math.toRadians(-90))
                                .splineTo(new Vector2d(sample2Ax,sample2Ay),Math.toRadians(sample2Ah))
                                .build(),
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.6).build(),
                                positionIntake(),
                                robot.intakeInferior.linearHorizontalMotor.goToExtended()
                        )
                )
                // todo; COLETAR SAMPLE
               ,
                robot.intakeInferior.intakeSuccao.GotoIntakeSpecimen(),

                new InstantAction(()-> {
                    robot.intakeInferior.intakeSuccao.sugador.setPower(0.8);
                }),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.6).build(),
                // todo; EXPULSAR SAMPLE
                robot.intakeInferior.intakeSuccao.GotoReadyToIntakeSpecimen(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                goToEjectSample(sample2Ax,sample2Ay,sample2Ah,sample2Bh),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                new ParallelAction(
                    robot.intakeInferior.intakeSuccao.IntakeRepelir(),
                        robot.intakeInferior.intakeSuccao.GotoIntakeSpecimen(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build()

                 )
        );

    }
    public Action collectSample3() {//todo okey
        return new SequentialAction(
                // todo; COLETAR SAMPLE
                new ParallelAction(
                        robot.md.actionBuilder(new Pose2d(sample2Ax, sample2Ay,Math.toRadians(sample2Bh))).waitSeconds(0.2)
                                // x maior para coletar
                                .strafeToLinearHeading(new Vector2d(sample3Ax, sample3Ay), Math.toRadians(sample3Ah))
                                .build(),
                        new SequentialAction(
                                robot.intakeInferior.intakeSuccao.GotoReadyToIntakeSpecimen(),
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                                robot.intakeInferior.linearHorizontalMotor.goToExtended()
                        )
                ),
                robot.intakeInferior.intakeSuccao.GotoIntakeSpecimen(),
                new InstantAction(()-> {
                    robot.intakeInferior.intakeSuccao.sugador.setPower(0.8);
                }),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                // todo; EXPULSAR SAMPLE
                robot.intakeInferior.intakeSuccao.GotoReadyToIntakeSpecimen(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                goToEjectSample(sample3Ax,sample3Ay,sample3Ah,sample3Bh),
                new InstantAction(()-> {
                    robot.intakeInferior.intakeSuccao.sugador.setPower(-0.8);
                }),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.5).build(),
                robot.intakeInferior.intakeSuccao.GotoReadyToIntakeSpecimen(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                robot.intakeInferior.intakeSuccao.GotoIntakeSpecimen()
        );
    }
    public Action collectSample4() {
        return new SequentialAction(
                // todo; COLETAR SAMPLE
                robot.md.actionBuilder(new Pose2d(sample3Ax, sample3Ay,Math.toRadians(sample3Bh))).waitSeconds(0.2)
                        .splineToLinearHeading(new Pose2d(sample4Ax, sample4Ay, Math.toRadians(sample4Ah)), Math.toRadians(0))
                        .build(),
                robot.intakeInferior.intakeSuccao.GotoReadyToIntakeSpecimen(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                robot.intakeInferior.linearHorizontalMotor.goToExtended(),
                robot.intakeInferior.intakeSuccao.GotoIntakeSpecimen(),

                new InstantAction(()-> {
                    robot.intakeInferior.intakeSuccao.sugador.setPower(0.8);
                }),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                // todo; EXPULSAR SAMPLE
                robot.intakeInferior.intakeSuccao.GotoReadyToIntakeSpecimen(),
                goToEjetingSample3(sample4Ax,sample4Ay,sample4Ah)
        );
    }
}
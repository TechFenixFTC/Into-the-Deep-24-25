package org.firstinspires.ftc.teamcode.opmode.v5_opModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;
import org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Inferior.UnderGrounSubystemStates;
import org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Superior.UpperSubsystemStates;


@Config
@Autonomous(name = "TesteIntake", group = "Autonomous")
public class AutSample0mais4 extends LinearOpMode {
    V5 robot;
    Action push;
    public static double
            Xdeposit = -51 ,    Ydeposit = -54 ,  Hdeposit = 45,
            XCollect2 = -47.5 , YCollect2 = -50 , HCollect2 = 90,
            XCollect3 = -58 ,   YCollect3 = -50 , HCollect3 = 90,
            XCollect4 = -46 ,   YCollect4 = -48 , HCollect4 = 135,
            Xpark = 0 , Ypark = 0

            ;


    ;


    public static double[] sample4 ={10,10,10};


    @Override
    public void runOpMode()  {

        Pose2d initialPose = new Pose2d(-35,  -62, Math.toRadians(0));

        robot = new V5(hardwareMap,telemetry);
        robot.md.pose = initialPose;
        Actions.runBlocking(
                new SequentialAction(
                        robot.outtakeIntakeSuperior.garraSuperior.fecharGarra(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        robot.outtakeIntakeSuperior.braco.goToInital(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        new InstantAction(() -> {robot.outtakeIntakeSuperior.garraSuperior.fecharGarra();}),
                        new InstantAction(() -> {robot.outtakeIntakeSuperior.garraSuperior.servoAngulacaoGarra.setPosition(0.625);}),
                        new InstantAction(() -> {robot.outtakeIntakeSuperior.garraSuperior.garraChamber();}),
                        new InstantAction(() -> {robot.intakeInferior.intakeSuccao.angulacao.setPosition(0.033);})
                )
        );


        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        //todo action que fica sempre rodando as ações com base nos estados passados
                        AutomationSample(),
                        new SequentialAction(
                            //todo : mudar estados do robo para outtake
                            new InstantAction(() -> robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INITIAL),
                            new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.OUTTAKE),
                            //todo ir até a basket para depositar
                            goToDeposit(),

                            //todo depositar
                            robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                            new InstantAction(() -> {robot.outtakeIntakeSuperior.garraSuperior.abrirGarra();}),

                            //todo depois de largar o sample esperar um pouco para ir para pegar o proximo sample(obs: criar uma action de abrir garra com tempo)
                            robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                            GoCollectSample2(),

                            //todo mudar estados para intake transfer para pegar o proximo sample
                            new InstantAction(() -> robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE),
                            new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.TRANSFER)

                        )

                )
        );

    }


    public Action AutomationSample(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.runStatesSampleAutonomo(robot.carteiro, getRuntime(),robot);
                return true;
            }
        };
    }

    public Action goToDeposit(){//todo okey
        return robot.md.actionBuilder(robot.md.pose)

                // basket
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(Xdeposit, Ydeposit, Math.toRadians(Hdeposit)), Math.toRadians(-135))
                .build();

    }


    public Action GoCollectSample2() {//todo okey
        return robot.md.actionBuilder(robot.md.pose)

                // collect Sample 2
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(XCollect2, YCollect2, Math.toRadians(HCollect2)), Math.toRadians(90))
                .build();

    }

    public Action GoCollectSample3() {//todo okey
        return robot.md.actionBuilder(robot.md.pose)

                // collect Sample 3
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(XCollect3, YCollect3, Math.toRadians(HCollect3)), Math.toRadians(135))
                .build();

    }
    public Action GoCollectSample4() {//todo okey
        return robot.md.actionBuilder(robot.md.pose)

                // collect Sample 4
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(XCollect4, YCollect4, Math.toRadians(HCollect4)), Math.toRadians(0))
                .build();
    }
}
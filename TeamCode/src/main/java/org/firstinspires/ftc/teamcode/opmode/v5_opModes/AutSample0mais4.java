package org.firstinspires.ftc.teamcode.opmode.v5_opModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.LinearVertical.LinearVertical;
import org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Inferior.UnderGrounSubystemStates;
import org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Superior.UpperSubsystemStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraOpeningStates;
import org.opencv.core.Mat;


@Config
@Autonomous(name = "Autonomo0+4", group = "Autonomous")
public class AutSample0mais4 extends LinearOpMode {
    V5 robot;
    Action push;
    public boolean encerrar = false;
    public static double
            Xdeposit = -57 ,    Ydeposit = -57 ,  Hdeposit = 45,
            XCollect2 = -47.5 , YCollect2 = -45 , HCollect2 = 100,
            XCollectParte1 = -52 , YCollectParte1 = -54 , HCollectParte1 = 95,
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
                        new InstantAction(() -> {robot.intakeInferior.intakeSuccao.angulacao.setPosition(0.033);}),
                        //todo : mudar estados do robo para outtake
                        new InstantAction(() -> robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INITIAL),
                        new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.OUTTAKE)
                )
        );


        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        //todo action que fica sempre rodando as ações com base nos estados passados
                        AutomationSample(),
                        new SequentialAction(
                                goToDeposit(),
                                esperarPraSoltarSample(),
                                new ParallelAction(
                                        new SequentialAction(
                                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.5).build(),
                                                new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL)
                                        ),
                                        GoCollectSample2()
                                ),

                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.7).build(),
                                new InstantAction(() -> robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE)

                            //todo ir até a basket para depositar
                            //goToDeposit(),
                            //todo abrir garra
                            //robot.md.actionBuilder(robot.md.pose).waitSeconds(0.9).build(),
                            //robot.outtakeIntakeSuperior.garraSuperior.abrirGarra(),
                            //esperarPraSoltarSample(),
                            //new InstantAction(() -> robot.carteiro.addOrder(robot.outtakeIntakeSuperior.garraSuperior.abrirGarra(),0,"abre caramba",getRuntime())) ,
                            /*robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                            //GoCollectPartido(),
                            //robot.md.actionBuilder(robot.md.pose).waitSeconds(0.3).build(),

                            //new InstantAction(() -> robot.carteiro.addOrder( new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL), 0, "superior pra initial", getRuntime())),
                            new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL),
                            robot.md.actionBuilder(robot.md.pose).waitSeconds(0.7).build(),
                            //GoCollectSample2(),
                            robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                            new InstantAction(() -> robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE),
                            //new InstantAction(() -> robot.carteiro.addOrder( new InstantAction(() -> robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE), 0, "superior pra intake", getRuntime())),
                            robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build()
                            //new InstantAction(() -> encerrar = true)

                            //todo depois de largar o sample esperar um pouco para ir para pegar o proximo sample(obs: criar uma action de abrir garra com tempo)
                            /*robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                            GoCollectSample2(),

                            //todo mudar estados para intake transfer para pegar o proximo sample
                            new InstantAction(() -> robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE),
                            new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.TRANSFER)*/

                        )

                )
        );

    }


    public Action AutomationSample(){
        return new Action() {
            boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started){
                    resetRuntime();
                }

                robot.outtakeIntakeSuperior.monitorEstadosAutonomo(telemetryPacket);
                robot.intakeInferior.monitorEstadosAutonomo(telemetryPacket);
                robot.intakeInferior.intakeSuccao.colorSensorSugar.colorMatcher.monitorAutonomo(telemetryPacket);
                robot.runStatesSampleAutonomo(robot.carteiro, getRuntime(),robot);

                if(encerrar){
                    return false;
                }
                return true;

            }
        };
    }

    public Action esperarPraSoltarSample(){
        return new Action() {
            boolean podeAbrirAGarra = false, jaChegouNoTarget = false, podeTerminarAaction;

            double tempoPraAbrirAgarra = 0.2, tempoPraTerminarAAction = tempoPraAbrirAgarra + 0.2;

            ElapsedTime tempoQueChegou = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {




                jaChegouNoTarget = robot.outtakeIntakeSuperior.linearVertical.chegouNoTargetAut();
                if(LinearVertical.targetPosition > 3000){
                    jaChegouNoTarget = robot.outtakeIntakeSuperior.linearVertical.motorR.getCurrentPosition() > 2500;
                }
                telemetryPacket.addLine("Vertical Chegou no target: "+ jaChegouNoTarget);
                telemetryPacket.addLine("TempoQueChegou: "+ tempoQueChegou.time());
                if(!jaChegouNoTarget) tempoQueChegou.reset();

                podeAbrirAGarra = tempoQueChegou.time() >= tempoPraAbrirAgarra;
                podeTerminarAaction = tempoQueChegou.time() >=tempoPraTerminarAAction;

                if(podeAbrirAGarra){
                    telemetryPacket.addLine("Garra ja abriu");
                    robot.outtakeIntakeSuperior.garraSuperior.garraOpeningState = GarraOpeningStates.OPEN;
                    robot.outtakeIntakeSuperior.garraSuperior.servoAberturaDaGarra.setPosition(robot.outtakeIntakeSuperior.garraSuperior.mapOpening.get(robot.outtakeIntakeSuperior.garraSuperior.garraOpeningState));
                }
                else{
                    telemetryPacket.addLine("Garra esperando para Abrir");
                }
                if(podeTerminarAaction){
                    telemetryPacket.addLine("Garra ja abriu");
                    return false;
                }



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

    public Action testIIntake(){
        return new InstantAction(()->{
            robot.carteiro.addOrder( new InstantAction(() -> robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE), 0, "inferior pra intake", getRuntime());
            robot.carteiro.addOrder(new InstantAction(()->  robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL), 0, "superior pra initial", getRuntime());
        });
    }
    public Action GoCollectSample2() {//todo okey
        return robot.md.actionBuilder(robot.md.pose)

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(XCollect2, YCollect2,Math.toRadians(HCollect2)), Math.toRadians(90))
                .build();
    }
    public Action GoCollectPartido() {//todo okey
        return robot.md.actionBuilder(robot.md.pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(XCollectParte1, YCollectParte1,Math.toRadians(HCollectParte1)), Math.toRadians(90))
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
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


@Config
@Autonomous(name = "Autonomo0+4", group = "Autonomous")
public class AutSample0mais4 extends LinearOpMode {
    V5 robot;
    Action push;
    public boolean encerrar = false;
    public static double
            Xdeposit1 = -59 ,    Ydeposit1 = -61.5 ,  Hdeposit1 = 45,
            Xdeposit2 = -54 ,    Ydeposit2 = -57 ,  Hdeposit2 = 45,
            XCollect2 = -45 , YCollect2 = -45, HCollect2 = 90,
            XCollectParte1 = -52 , YCollectParte1 = -54 , HCollectParte1 = 95,
            XCollect3 = -58 ,   YCollect3 = -50 , HCollect3 = 90,
            XCollect4 = -46 ,   YCollect4 = -48 , HCollect4 = 135,

            delaySoltarSample1 = 1.8, delaySoltarSample2 = 1.7, delayEsperarParaIrParaFrenteSample2 = 0.2,
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
                                goToDeposit(new Pose2d(-35,  -62, Math.toRadians(0))),
                                esperarPraSoltarSample(delaySoltarSample1),
                                new ParallelAction(
                                        new SequentialAction(
                                                new InstantAction(() -> robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE),
                                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.500).build(),
                                                new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL)

                                        ),
                                        GoCollectPartido()
                                ),
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(delayEsperarParaIrParaFrenteSample2).build(),
                                GoCollectSample2(),
                                esperarPeloInicioDoOutake(),

                            //todo ir até a basket para depositar o segundo sample
                            goToDeposit2(new Pose2d(XCollect2, YCollect2, HCollect2)),
                            //todo abrir garra
                            esperarPraSoltarSample(delaySoltarSample2),
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(30).build(),

                                new ParallelAction(
                                        new SequentialAction(
                                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.5).build(),
                                                new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL)
                                        ),
                                        GoCollectSample3() // collect sample 3
                                ),
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.7).build(),
                                new InstantAction(() -> robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE),
                                esperarPeloInicioDoOutake(),
                                //todo ir até a basket para depositar
                                goToDeposit(new Pose2d(XCollect3, YCollect3,HCollect3)),

                                //todo abrir garra
                                esperarPraSoltarSample(1.7),
                                new ParallelAction(
                                        new SequentialAction(
                                                robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                                                new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL)
                                        ),
                                        GoCollectSample3() // collect sample 3
                                ),
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.7).build(),
                                new InstantAction(() -> robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE),
                                esperarPeloInicioDoOutake(),
                                //todo ir até a basket para depositar
                                goToDeposit(new Pose2d(XCollect4, YCollect4,HCollect4)),
                                esperarPraSoltarSample(1.7)


                            //robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                            //new InstantAction(() -> encerrar = true)




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
                    started =true;
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

    public Action esperarPeloInicioDoOutake(){
        // verificar se um action já terminou para passar o estado para outra
        // por enquanto só para intake
        return new Action() {
            boolean OuttakeSuperiorFuncionou                  = false;
            boolean EstadoDosInferioresEstaInicial            = false;
            double tempoLimiteParaTerminarAAction = 10;

            boolean podeTerminar = false;
            ElapsedTime tempoAtual = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                OuttakeSuperiorFuncionou       = robot.outtakeIntakeSuperior.upperSubsystemStates == UpperSubsystemStates.OUTTAKE; // garra já abriu
                EstadoDosInferioresEstaInicial = robot.intakeInferior.underGrounSubystemStates == UnderGrounSubystemStates.INITIAL; // se ele realmente funcionou, ele transiciona para initial pq o já pegou o sample

                if(OuttakeSuperiorFuncionou && EstadoDosInferioresEstaInicial){
                    return  false;
                }
                podeTerminar = tempoAtual.time() > tempoLimiteParaTerminarAAction;
                if(podeTerminar){
                    return false;
                }
                return true ;
            }
        };
    }
    public Action esperarPraSoltarSample(double delay){
        return new Action() {
            boolean podeAbrirAGarra = false, jaChegouNoTarget = false, podeTerminarAaction;

            double tempoPraAbrirAgarra = delay, tempoPraTerminarAAction = tempoPraAbrirAgarra + 0.2;
            // aumeentar um pouquinho

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
    public Action goToDeposit(Pose2d pose2d){//todo okey
        return robot.md.actionBuilder(pose2d)
                // basket
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(Xdeposit1, Ydeposit1, Math.toRadians(Hdeposit1)), Math.toRadians(-135))
                .build();

    }
    public Action goToDeposit2(Pose2d pose2d){//todo okey
        return robot.md.actionBuilder(pose2d)
                // basket
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(Xdeposit2, Ydeposit2, Math.toRadians(Hdeposit2)), Math.toRadians(-135))
                .build();

    }

    public Action testIIntake(){
        return new InstantAction(()->{
            robot.carteiro.addOrder( new InstantAction(() -> robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE), 0, "inferior pra intake", getRuntime());
            robot.carteiro.addOrder(new InstantAction(()->  robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL), 0, "superior pra initial", getRuntime());
        });
    }
    public Action GoCollectSample2() {//todo okey
        return robot.md.actionBuilder(new Pose2d(XCollect2,YCollect2 -3, HCollect2))

                //.setTangent(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(XCollect2,YCollect2), Math.toRadians(HCollect2))
                .build();
    }

    public Action GoCollectPartido() {//todo okey
        return robot.md.actionBuilder(new Pose2d(Xdeposit1, Ydeposit1, Hdeposit1))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(XCollect2, YCollect2 - 3,Math.toRadians(HCollect2)), Math.toRadians(90))
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
package org.firstinspires.ftc.teamcode.robot.Controller;

import static org.firstinspires.ftc.teamcode.opmode.v5_opModes.autonomous.AutSample0mais4Red.YCollect2;
import static org.firstinspires.ftc.teamcode.opmode.v5_opModes.autonomous.AutSample0mais4Red.tangentCollect4;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.V5;
import org.firstinspires.ftc.teamcode.opmode.v5_opModes.autonomous.AutSample0mais4Red;
import org.firstinspires.ftc.teamcode.opmode.v5_opModes.autonomous.AutoSpecimen5mais0Sugando;
import org.firstinspires.ftc.teamcode.common.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.Horizontal.LinearHorizontalStates;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.Sugar.IntakeSuccao;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.UnderGrounSubystemStates;
import org.firstinspires.ftc.teamcode.robot.subsistemas.common.Garra.GarraOpeningStates;

import java.util.ArrayList;

public class Controladora5mais0_sugando {
    double limelightCorrection = 0;
    public AutonomoStates estadoSample = AutonomoStates.Initial;
    public int sampleID = 1, samplesNotCollected = 0;
    ArrayList<Integer> samplesIdNotCollected = new ArrayList<>();


    public int quantasVezesFoiExecutado = 0;

    // todo: verificar estados
    public Controladora5mais0_sugando() {

    }
    public static V5 create5mais0Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        Pose2d initialPose =  new Pose2d(10, -60, Math.toRadians(90));
        V5 robot = new V5(hardwareMap,telemetry);
        robot.md.pose = initialPose;
        MecanumDrive.PARAMS.maxProfileAccel = 80;
        MecanumDrive.PARAMS.minProfileAccel = -80;
        MecanumDrive.PARAMS.maxWheelVel  = 80;

        return robot;
    }

    public Action decideProximaAcao(V5 robot, double runtime){
        quantasVezesFoiExecutado++;
        robot.md.updatePoseEstimate();
        Action proximaAction = null;

        if(estadoSample == AutonomoStates.Initial && sampleID == 1 ) {//todo porque isso funciona?
            estadoSample = AutonomoStates.ReadyToCollect;
            proximaAction = goToReadyToCollect(sampleID,robot);
        }
        else if(estadoSample == AutonomoStates.ReadyToCollect && sampleID >= 2){
            estadoSample = AutonomoStates.Collect;
            proximaAction = CollectSample(sampleID,robot);
        }
        else if(estadoSample == AutonomoStates.Collect && sampleID >= 2){
            estadoSample = AutonomoStates.Ejetar;
            proximaAction = entregar(sampleID, robot);
        }
        else if(estadoSample == AutonomoStates.Ejetar && sampleID >= 2){
            estadoSample = AutonomoStates.ReadyToCollect;
            proximaAction = goToReadyToCollect(sampleID, robot);
        }


        // todo: caso dê errado
        return proximaAction;
    }
    public Action entregar(int idSample, V5 robot){
        estadoSample = AutonomoStates.Ejetar;
        robot.md.updatePoseEstimate();

        if(idSample == 2){
            return entregarSample2(robot.md.pose, robot);
        }
        if(idSample == 3){
            return entregarSample3(robot.md.pose, robot);
        }
        if(idSample == 4){
            return entregarSample4(robot.md.pose, robot);
        }

        // default
        return new SequentialAction();
    }
    public Action CollectSample(int idSample, V5 robot){
        estadoSample = AutonomoStates.Collect;
        robot.md.updatePoseEstimate();
        if(idSample == 2){
            return coletarSample2(robot.md.pose, robot);
        }
        if(idSample == 3){
            return coletarSample3(robot.md.pose, robot);
        }
        if(idSample == 4){
            return coletarSample4(robot.md.pose, robot);
        }
        // default
        return new SequentialAction();
    }

    public Action goToReadyToCollect(int idSample, V5 robot){ /*todo: para quando não conseguir pegar nenhuma sample, ir pra outra*/
        estadoSample = AutonomoStates.ReadyToCollect;

        robot.md.updatePoseEstimate();
        if(idSample == 2){
            return preparaPraColetarSample2(robot.md.pose, robot);
        }
        if(idSample == 3){
            return preparaPraColetarSample3(robot.md.pose, robot);
        }
        if(idSample == 4){
            return preparaPraColetarSample4(robot.md.pose, robot);
        }
        // default
        return new SequentialAction();
    }


/************************************\
*   Ações específicas do 5 mais 0
\*************************************/


    /******************************\
     *   Wait To Condition
     \*****************************/
    public  Action esperarPeloInicioDoEntregar(double tempoLimite, V5 robot){
        // adaptação do esperarPelInicioDoOuttake para funcionar cm o intake specimen
        // por enquanto só para intake
        return new Action() {

            boolean SampleFicouAgarrada                       = false;

            boolean temSampleMasNaoTaBoaParaPegar             = robot.intakeInferior.intakeSuccao.colorSensorSugar.colorMatcher.temUmaSampleNoIntake() && !robot.intakeInferior.intakeSuccao.colorSensorSugar.colorMatcher.sampleNaPosicaoCorreta();

            boolean IntakeForcando                            = (robot.intakeInferior.intakeSuccao.sugador.getCurrent(CurrentUnit.AMPS) > AutSample0mais4Red.currMax);
            double tempoLimiteParaTerminarAAction = tempoLimite;

            double lembrarPowerSugador = IntakeSuccao.power_Sugador;
            boolean podeTerminar = false, started = false;
            ElapsedTime tempoAtual = new ElapsedTime(), tempoForcando = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetryPacket.addLine("Tempo: "+ tempoAtual.time());
                telemetryPacket.put("corrente sugador", robot.intakeInferior.intakeSuccao.sugador.getCurrent(CurrentUnit.AMPS));
                if(!started){
                    tempoAtual.reset();
                    started = true;
                }


                //todo primeira condição para terminar (intake forçando)
                if(IntakeForcando) {
                    podeTerminar = true;
                }

                //todo segunda condição para terminar (detecção de sample)
                if(temSampleMasNaoTaBoaParaPegar){
                    podeTerminar = true;
                }
                //todo terceira condição pra terminar (tempo)
                podeTerminar = tempoAtual.time() > tempoLimiteParaTerminarAAction;


                if(podeTerminar){
                    if (sampleID < 4) robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE;
                    else robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INITIAL;
                    IntakeSuccao.power_Sugador = lembrarPowerSugador;
                    return false;
                }
                return true ;
            }
        };
    }
    public Action esperarPraSoltarSample(double delay, V5 robot){
        return new Action() {
            boolean podeAbrirAGarra = false, jaChegouNoTarget = false, podeTerminarAaction = false, started = false;

            double tempoPraAbrirAgarra = delay;
            double tempoPraTerminarAAction = tempoPraAbrirAgarra + 0.22;
            // aumeentar um pouquinho

            final ElapsedTime tempoQueChegou = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetryPacket.addLine("COMEÇANDO A ESPERAR");
                if(!started) {
                    tempoQueChegou.reset();
                    started = true;
                }


                jaChegouNoTarget = robot.outtakeIntakeSuperior.linearVertical.motorR.getCurrentPosition() > 2500;

                if(!jaChegouNoTarget) tempoQueChegou.reset();

                telemetryPacket.addLine("Vertical Chegou no target: "+ jaChegouNoTarget);
                telemetryPacket.addLine("TempoQueChegou: "+ tempoQueChegou.time());

                podeAbrirAGarra = tempoQueChegou.time() >= tempoPraAbrirAgarra;
                podeTerminarAaction = tempoQueChegou.time() >= tempoPraTerminarAAction;

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
    public Action esperarPeloOutakeTerSubidoEextender(double delay, V5 robot){
        return new Action() {
            boolean jaSubiu = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetryPacket.addLine("Esperando pra extender");

                jaSubiu = robot.outtakeIntakeSuperior.linearVertical.motorR.getCurrentPosition() > 1200;

                if(jaSubiu){
                    telemetryPacket.addLine("Extendendo");
                    if(sampleID < 4) {
                        robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE;
                    }else{
                        robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.READY_TOINTAKE;
                    }

                    return false;
                }



                return true;
            }
        };
    }

    /************************************\
     *   Trajetória das Entregas
     \*************************************/
    public Action entregarSample2(Pose2d pose2d, V5 robot){
        Action move = robot.md.actionBuilder(robot.md.pose)
                .splineToLinearHeading(new Pose2d(AutoSpecimen5mais0Sugando.XEntregarSample2,AutoSpecimen5mais0Sugando.YEntregarSample2,Math.toRadians(AutoSpecimen5mais0Sugando.HEntregarSample2)),Math.toRadians(-90))
                .build();
        return  new SequentialAction(
                new InstantAction(()->{estadoSample = AutonomoStates.Collect;}),
                move,
                new InstantAction(() -> sampleID+= 1)

        );

    }
    public Action entregarSample3(Pose2d pose2d, V5 robot){
        Action move = robot.md.actionBuilder(robot.md.pose)
                .splineToLinearHeading(new Pose2d(AutoSpecimen5mais0Sugando.XEntregarSampl3,AutoSpecimen5mais0Sugando.YEntregarSample3,Math.toRadians(AutoSpecimen5mais0Sugando.HEntregarSample3)),Math.toRadians(-90))
                .build();
        return  new SequentialAction(
                new InstantAction(()->{estadoSample = AutonomoStates.Collect;}),
                move,
                new InstantAction(() -> sampleID+= 1)
        );

    }
    public Action entregarSample4(Pose2d pose2d , V5 robot) {
        Action move = robot.md.actionBuilder(robot.md.pose)
                .splineToLinearHeading(new Pose2d(AutoSpecimen5mais0Sugando.XEntregarSample4,AutoSpecimen5mais0Sugando.YEntregarSample4,Math.toRadians(AutoSpecimen5mais0Sugando.HEntregarSample4)),Math.toRadians(-90))
                .build();
        return  new SequentialAction(
                new InstantAction(()->{estadoSample = AutonomoStates.Collect;}),
                move,
                new InstantAction(() -> sampleID+= 1)
        );
    }

     /*****************************************\
     * Trajetórias - GoToReadyToCollect (Padrão)
     \*****************************************/
    public Action preparaPraColetarSample2(Pose2d pose2d, V5 robot){
        Action move = robot.md.actionBuilder(robot.md.pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(AutoSpecimen5mais0Sugando.XReadCollect2,AutoSpecimen5mais0Sugando.YReadCollect2,Math.toRadians(AutoSpecimen5mais0Sugando.HReadCollect2)),Math.toRadians(90))
                //.waitSeconds(5)
                .build();

        return new SequentialAction(
                new InstantAction(() -> robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE),
                move
        );

    }
    public Action preparaPraColetarSample3(Pose2d pose2d, V5 robot){
        Action move = robot.md.actionBuilder(robot.md.pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(AutoSpecimen5mais0Sugando.XReadCollect3,AutoSpecimen5mais0Sugando.YReadCollect3,Math.toRadians(AutoSpecimen5mais0Sugando.HReadCollect3)),Math.toRadians(90))
                .build();
        return  new SequentialAction(


        );

    }
    public Action preparaPraColetarSample4(Pose2d pose2d, V5 robot){
        Action move = robot.md.actionBuilder(robot.md.pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(AutoSpecimen5mais0Sugando.XReadCollect4,AutoSpecimen5mais0Sugando.YReadCollect4,Math.toRadians(AutoSpecimen5mais0Sugando.HReadCollect4)),Math.toRadians(90))
                .build();
        return  new SequentialAction(


        );

    }

    /******************************************\
     *   Trajetórias - GoToCollect
     \*******************************************/
    public Action coletarSample2(Pose2d pose2d, V5 robot){

        Action move = robot.md.actionBuilder(robot.md.pose)
                .splineToSplineHeading(new Pose2d(AutoSpecimen5mais0Sugando.XCollect2, AutoSpecimen5mais0Sugando.YCollect2 - 4, Math.toRadians(AutoSpecimen5mais0Sugando.HCollect2 + 7)), Math.toRadians(90),null, new ProfileAccelConstraint(-10, 10))
                .splineToSplineHeading(new Pose2d(AutoSpecimen5mais0Sugando.XCollect2, AutoSpecimen5mais0Sugando.YCollect2 - 2, Math.toRadians(AutoSpecimen5mais0Sugando.HCollect2 - 7)), Math.toRadians(90),null, new ProfileAccelConstraint(-10, 10))
                .splineToSplineHeading(new Pose2d(AutoSpecimen5mais0Sugando.XCollect2, AutoSpecimen5mais0Sugando.YCollect2, Math.toRadians(AutoSpecimen5mais0Sugando.HCollect2)), Math.toRadians(90),null, new ProfileAccelConstraint(-10, 10))
                .strafeTo(new Vector2d(AutoSpecimen5mais0Sugando.XCollect2+7 ,YCollect2))

                //.waitSeconds(5)
                .build();
        return  new SequentialAction(
                new InstantAction(()->{estadoSample = AutonomoStates.Collect;}),
                move,
                esperarPeloInicioDoEntregar(2.5,robot)

        );


    }
    public Action coletarSample3(Pose2d pose2d, V5 robot){

        Action move = robot.md.actionBuilder(robot.md.pose)
                .splineToSplineHeading(new Pose2d(AutoSpecimen5mais0Sugando.XCollect3, AutoSpecimen5mais0Sugando.YCollect3 - 4, Math.toRadians(AutoSpecimen5mais0Sugando.HCollect3 + 2)), Math.toRadians(90),null, new ProfileAccelConstraint(-10, 10))
                .splineToSplineHeading(new Pose2d(AutoSpecimen5mais0Sugando.XCollect3, AutoSpecimen5mais0Sugando.YCollect3 - 2, Math.toRadians(AutoSpecimen5mais0Sugando.HCollect3 - 2)), Math.toRadians(90),null, new ProfileAccelConstraint(-10, 10))
                .splineToSplineHeading(new Pose2d(AutoSpecimen5mais0Sugando.XCollect3, AutoSpecimen5mais0Sugando.YCollect3, Math.toRadians(AutoSpecimen5mais0Sugando.HCollect3)), Math.toRadians(90),null, new ProfileAccelConstraint(-10, 10))
                .build();
        return  new SequentialAction(
                new InstantAction(()->{estadoSample = AutonomoStates.Collect;}),
                move,
                esperarPeloInicioDoEntregar(2.5,robot)
        );

    }
    public Action coletarSample4(Pose2d pose2d, V5 robot){

        Action move = robot.md.actionBuilder(robot.md.pose)
                .splineToSplineHeading(new Pose2d(AutoSpecimen5mais0Sugando.XCollect4, AutoSpecimen5mais0Sugando.YCollect4 - 4, Math.toRadians(AutoSpecimen5mais0Sugando.HCollect4 + 2)), Math.toRadians(90),null, new ProfileAccelConstraint(-10, 10))
                .splineToSplineHeading(new Pose2d(AutoSpecimen5mais0Sugando.XCollect4, AutoSpecimen5mais0Sugando.YCollect4 - 2, Math.toRadians(AutoSpecimen5mais0Sugando.HCollect4 - 2)), Math.toRadians(90),null, new ProfileAccelConstraint(-10, 10))
                .splineToSplineHeading(new Pose2d(AutoSpecimen5mais0Sugando.XCollect4, AutoSpecimen5mais0Sugando.YCollect4, Math.toRadians(AutoSpecimen5mais0Sugando.HCollect4)), Math.toRadians(90),null, new ProfileAccelConstraint(-10, 10))
                .build();
        return  new SequentialAction(
                new InstantAction(()->{estadoSample = AutonomoStates.Collect;}),
                move,
                esperarPeloInicioDoEntregar(2.5,robot)
        );

    }

    //antiga
    public Action GoToCollectSample2(Pose2d pose2d, V5 robot) {
        Action move = robot.md.actionBuilder(pose2d)
                .splineToSplineHeading(new Pose2d(35, -40 - 4, Math.toRadians(60 + 2)), Math.toRadians(90))//todo -10
                .splineToSplineHeading(new Pose2d(35, -40 - 2, Math.toRadians(60 - 2)), Math.toRadians(90))//todo -10
                .splineToSplineHeading(new Pose2d(35, -40, Math.toRadians(60)), Math.toRadians(90))//todo -10
                .build();

        return new SequentialAction(
                move,
                esperarPeloInicioDoEntregar(2.5, robot)
        );
    }
    public Action GoToCollectSample3(Pose2d pose2d, V5 robot) {
        Action move = robot.md.actionBuilder(pose2d)
                .splineToSplineHeading(new Pose2d(AutSample0mais4Red.XCollect3 + limelightCorrection, AutSample0mais4Red.YCollect3 - 4, Math.toRadians(AutSample0mais4Red.HCollect3 + 7)), Math.toRadians(90), null, new ProfileAccelConstraint(-10, 10))
                .splineToSplineHeading(new Pose2d(AutSample0mais4Red.XCollect3 + limelightCorrection, AutSample0mais4Red.YCollect3 - 2, Math.toRadians(AutSample0mais4Red.HCollect3 - 7)), Math.toRadians(90), null, new ProfileAccelConstraint(-10, 10))
                .splineToSplineHeading(new Pose2d(AutSample0mais4Red.XCollect3 + limelightCorrection, AutSample0mais4Red.YCollect3, Math.toRadians(AutSample0mais4Red.HCollect3)), Math.toRadians(90), null, new ProfileAccelConstraint(-10, 10))
                .build();

        return new SequentialAction(
                move,
                esperarPeloInicioDoEntregar(2.5, robot)
        );
    }
    public Action GoToCollectSample4(Pose2d pose2d, V5 robot) {
        Action move = robot.md.actionBuilder(pose2d)
                //.splineToSplineHeading(new Pose2d(AutSample0mais4Red.XCollect4, AutSample0mais4Red.YCollect4 - 4, Math.toRadians(AutSample0mais4Red.HCollect4)), Math.toRadians(tangentCollect4),null, new ProfileAccelConstraint(-50, 50))
                //.splineToSplineHeading(new Pose2d(AutSample0mais4Red.XCollect4, AutSample0mais4Red.YCollect4 - 2, Math.toRadians(AutSample0mais4Red.HCollect4 + 10)), Math.toRadians(tangentCollect4),null, new ProfileAccelConstraint(-10, 10))
                //todo: ver a poição do 4 porque esta travando toda hr
                .splineToSplineHeading(new Pose2d(AutSample0mais4Red.XCollect4, AutSample0mais4Red.YCollect4, Math.toRadians(AutSample0mais4Red.HCollect4)), Math.toRadians(tangentCollect4),null, new ProfileAccelConstraint(-80, 80))
                //.splineToSplineHeading(new Pose2d(AutSample0mais5.XCollect4 - 2,AutSample0mais5.YCollect4, Math.toRadians(AutSample0mais5.HCollect4 - 30)), Math.toRadians(tangentCollect4 - 40))
                .build();

        return new SequentialAction(
                move,
                esperarPeloInicioDoEntregar(2.5, robot)
        );
    }

    /************************************\
     *        Wait To Condition
     \***********************************/
    public Action esperarLevantarAngulacao(double runtime,double tempolimite,V5 robot){
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                boolean horizontalEstaRetraido = robot.intakeInferior.linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.EXTENDED;
                boolean intakeFuncionou        = robot.controladoraSPECIMEN.estadoSample == AutonomoStates.Ejetar;
                boolean podeDescerAngulcao     = robot.controladoraSPECIMEN.estadoSample == AutonomoStates.Collect;
                boolean podeSubirAngulacao     = robot.controladoraSPECIMEN.estadoSample == AutonomoStates.Ejetar;

                if(podeDescerAngulcao){
                    //todo agulação
                    //robot.intakeInferior.intakeSuccao.angulacao.setPosition(robot.intakeInferior.intakeSuccao.mapAngulation);


                }
                if(podeSubirAngulacao){
                    if(robot.carteiro.hasOrder("angulacaoReadyInitial")){
                        robot.carteiro.addOrder(
                                robot.intakeInferior.intakeSuccao.GotoReadyToIntakeSample(),
                                0,
                                "angulacaoReadyIntake",
                                runtime
                        );
                        robot.intakeInferior.estaEjetando = true;
                    }
                }


                return true;
            }
        };
    }




    /************************************\
     *           Verificações
     \**********************************/

    public Action LimelightCalculateCorrection(double targetZ, V5 robot) {
        return new Action() {
            final ElapsedTime temporizador = new ElapsedTime();
            boolean started = false;

            double currentZ = 0, erro = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Inicia o cronômetro na primeira execução
                if (!started) {
                    temporizador.reset();
                    started = true;
                }
                currentZ = robot.limelight.getZdistanceFromAprilTag();

                // Calcula o erro de posição
                if(currentZ != 0){
                    erro = targetZ - currentZ;
                    limelightCorrection = erro * AutSample0mais4Red.LimeLightKp;
                }
                // Define um fator de correção proporcional (ajuste conforme necessário)

                // Enviar informações para telemetria
                telemetryPacket.put("Limelight Z", currentZ);
                telemetryPacket.put("Erro Z", erro);
                telemetryPacket.addLine("limelightCorrection("+limelightCorrection+" = ( targetZ("+targetZ+") - currentZ("+currentZ+") ) * kp("+ AutSample0mais4Red.LimeLightKp+")");
                telemetryPacket.put("Correção Aplicada", limelightCorrection);

                // Se o erro for pequeno o suficiente ou o tempo limite for atingido, finaliza a ação
                if ( temporizador.time() >= AutSample0mais4Red.LimelightDelayToRead) {
                    if(Math.abs(erro) < 0.02 || Math.abs(erro) > 1 || erro > 0.5) limelightCorrection = 0;
                    return false;
                }

                return true;
            }
        };
    }

}

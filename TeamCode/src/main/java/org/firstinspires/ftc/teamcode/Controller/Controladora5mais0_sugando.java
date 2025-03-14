package org.firstinspires.ftc.teamcode.Controller;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;
import org.firstinspires.ftc.teamcode.opmode.v5_opModes.AutSample0mais4Red;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal.LinearHorizontalStates;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar.IntakeSuccao;
import org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Inferior.UnderGrounSubystemStates;
import org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Superior.UpperSubsystemStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraOpeningStates;

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
        Pose2d initialPose = new Pose2d(-35,  -62, Math.toRadians(0));
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
        if(estadoSample == AutonomoStates.Initial && sampleID == 1 ){
            //todo adicionar ID
            proximaAction = goToReadyToCollect(2,robot);
        }
        else if(estadoSample == AutonomoStates.ReadyToCollect && sampleID >= 2){
            proximaAction = CollectSample(sampleID,robot);
        }
        else if(estadoSample == AutonomoStates.Collect && sampleID >= 2){
            proximaAction = entregar(sampleID, robot);
        }
        else if(estadoSample == AutonomoStates.Deposit && sampleID >= 2){
            proximaAction = goToReadyToCollect(sampleID, robot);
        }


        // todo: caso dê errado
        return proximaAction;
    }
    public Action entregar(int idSample, V5 robot){
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
    /************************************************\
     *     Preparar pra  Coletar
     \*************************************************/
    public Action preparaPraColetarSample2(Pose2d pose2d, V5 robot){//todo okey
        return  new SequentialAction(
                goToDeposit2(pose2d, robot),
                //todo abrir garra
                new ParallelAction(
                        esperarPraSoltarSample(AutSample0mais4Red.delaySoltarSample2, robot),
                        new SequentialAction(
                                esperarPeloOutakeTerSubidoEextender(0, robot)
                        )
                ),
                new InstantAction(() -> sampleID+= 1),
                new ParallelAction(

                        new InstantAction(() -> estadoSample = AutonomoStates.ReadyToCollect),
                        GoToReadyToCollectSample3(robot.md.pose, robot),
                        new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL)
                )
        );

    }
    public Action preparaPraColetarSample3(Pose2d pose2d, V5 robot){//todo okey
        return  new SequentialAction(
                goToDeposit2(pose2d, robot),
                //todo abrir garra
                new ParallelAction(
                        esperarPraSoltarSample(AutSample0mais4Red.delaySoltarSample2, robot),
                        new SequentialAction(
                                esperarPeloOutakeTerSubidoEextender(0, robot)
                        )
                ),
                new InstantAction(() -> sampleID+= 1),
                new ParallelAction(
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.500).build(),
                                new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL)
                        ),
                        new InstantAction(() -> estadoSample = AutonomoStates.ReadyToCollect),
                        GoToReadyToCollectSample4(robot.md.pose, robot)
                )
        );

    }
    public Action preparaPraColetarSample4(Pose2d pose2d, V5 robot){//todo okey
        return  new SequentialAction(
                goToDeposit4(pose2d, robot),
                //todo abrir garra
                new ParallelAction(
                        esperarPraSoltarSample(AutSample0mais4Red.delaySoltarSample4, robot),
                        new SequentialAction(
                                // esperarPeloOutakeTerSubidoEextender(1, robot)
                        )
                ),
                new InstantAction(() -> sampleID+= 1),
                new ParallelAction(
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.500).build(),
                                new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL)
                        ),
                        new InstantAction(() -> estadoSample = AutonomoStates.ReadyToCollect)
                )
        );

    }
    /************************************************\
     *       Coletar
     \*************************************************/
    public Action coletarSample2(Pose2d pose2d, V5 robot){//todo okey
        return  new SequentialAction(
                goToDeposit2(pose2d, robot),
                //todo abrir garra
                new ParallelAction(
                        esperarPraSoltarSample(AutSample0mais4Red.delaySoltarSample2, robot),
                        new SequentialAction(
                                esperarPeloOutakeTerSubidoEextender(0, robot)
                        )
                ),
                new InstantAction(() -> sampleID+= 1),
                new ParallelAction(

                        new InstantAction(() -> estadoSample = AutonomoStates.ReadyToCollect),
                        GoToReadyToCollectSample3(robot.md.pose, robot),
                        new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL)
                )
        );

    }
    public Action coletarSample3(Pose2d pose2d, V5 robot){//todo okey
        return  new SequentialAction(
                goToDeposit2(pose2d, robot),
                //todo abrir garra
                new ParallelAction(
                        esperarPraSoltarSample(AutSample0mais4Red.delaySoltarSample2, robot),
                        new SequentialAction(
                                esperarPeloOutakeTerSubidoEextender(0, robot)
                        )
                ),
                new InstantAction(() -> sampleID+= 1),
                new ParallelAction(
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.500).build(),
                                new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL)
                        ),
                        new InstantAction(() -> estadoSample = AutonomoStates.ReadyToCollect),
                        GoToReadyToCollectSample4(robot.md.pose, robot)
                )
        );

    }
    public Action coletarSample4(Pose2d pose2d, V5 robot){//todo okey
        return  new SequentialAction(
                goToDeposit4(pose2d, robot),
                //todo abrir garra
                new ParallelAction(
                        esperarPraSoltarSample(AutSample0mais4Red.delaySoltarSample4, robot),
                        new SequentialAction(
                                // esperarPeloOutakeTerSubidoEextender(1, robot)
                        )
                ),
                new InstantAction(() -> sampleID+= 1),
                new ParallelAction(
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.500).build(),
                                new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL)
                        ),
                        new InstantAction(() -> estadoSample = AutonomoStates.ReadyToCollect)
                )
        );

    }
    /************************************************\
     *       Entregar ( Levar pra observation zone)
     \*************************************************/
    public Action entregarSample2(Pose2d pose2d, V5 robot){//todo okey
        return  new SequentialAction(
                goToDeposit2(pose2d, robot),
                //todo abrir garra
                new ParallelAction(
                        esperarPraSoltarSample(AutSample0mais4Red.delaySoltarSample2, robot),
                        new SequentialAction(
                                esperarPeloOutakeTerSubidoEextender(0, robot)
                        )
                ),
                new InstantAction(() -> sampleID+= 1),
                new ParallelAction(
                        
                        new InstantAction(() -> estadoSample = AutonomoStates.ReadyToCollect),
                        GoToReadyToCollectSample3(robot.md.pose, robot),
        new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL)
                )
        );

    }
    public Action entregarSample3(Pose2d pose2d, V5 robot){//todo okey
        return  new SequentialAction(
                goToDeposit2(pose2d, robot),
                //todo abrir garra
                new ParallelAction(
                        esperarPraSoltarSample(AutSample0mais4Red.delaySoltarSample2, robot),
                        new SequentialAction(
                                esperarPeloOutakeTerSubidoEextender(0, robot)
                        )
                ),
                new InstantAction(() -> sampleID+= 1),
                new ParallelAction(
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.500).build(),
                                new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL)
                        ),
                        new InstantAction(() -> estadoSample = AutonomoStates.ReadyToCollect),
                        GoToReadyToCollectSample4(robot.md.pose, robot)
                )
        );

    }
    public Action entregarSample4(Pose2d pose2d, V5 robot){//todo okey
        return  new SequentialAction(
                goToDeposit4(pose2d, robot),
                //todo abrir garra
                new ParallelAction(
                        esperarPraSoltarSample(AutSample0mais4Red.delaySoltarSample4, robot),
                        new SequentialAction(
                               // esperarPeloOutakeTerSubidoEextender(1, robot)
                        )
                ),
                new InstantAction(() -> sampleID+= 1),
                new ParallelAction(
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.500).build(),
                                new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL)
                        ),
                        new InstantAction(() -> estadoSample = AutonomoStates.ReadyToCollect)
                )
        );

    }

    /************************************\
     *   Wait To Condition
     \*************************************/
    public  Action esperarPeloInicioDoOutake(double tempoLimite, V5 robot){
        // verificar se um action já terminou para passar o estado para outra
        // por enquanto só para intake
        return new Action() {
            boolean OuttakeSuperiorFuncionou                  = false;
            boolean LinearHorizontalTaRetraido                = robot.intakeInferior.linearHorizontalMotor.linearHorizontalInferiorState ==LinearHorizontalStates.RETRACTED;
            boolean EstadoDosInferioresEstaInicial            =  robot.intakeInferior.underGrounSubystemStates == UnderGrounSubystemStates.INITIAL;
            boolean SampleFicouAgarrada                       = false;
            boolean naoTemSample                              = robot.intakeInferior.intakeSuccao.colorSensorSugar.colorMatcher.getSampleColor().equals("Não há samples no intake");
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
                OuttakeSuperiorFuncionou       = robot.outtakeIntakeSuperior.upperSubsystemStates == UpperSubsystemStates.OUTTAKE; // garra já abriu
                double tempoRestante           = tempoLimite - tempoAtual.time();

                if((temSampleMasNaoTaBoaParaPegar && LinearHorizontalTaRetraido  && EstadoDosInferioresEstaInicial) || (LinearHorizontalTaRetraido && EstadoDosInferioresEstaInicial && naoTemSample)){
                    if(tempoAtual.time() > 1.900){
                       // robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE;
                    }

                }
                if(IntakeForcando) {
                    if(tempoForcando.time() < 0.15){
                        IntakeSuccao.power_Sugador = 1;
                    }
                    else if(tempoForcando.time() < 0.35){
                        IntakeSuccao.power_Sugador = -0.2;
                    }else{
                        tempoForcando.reset();
                        IntakeSuccao.power_Sugador = lembrarPowerSugador;
                    }
                }else{
                    tempoForcando.reset();
                    IntakeSuccao.power_Sugador = lembrarPowerSugador;
                }

                if(OuttakeSuperiorFuncionou){
                    IntakeSuccao.power_Sugador = lembrarPowerSugador;
                    return  false;
                }

                podeTerminar = tempoAtual.time() > tempoLimiteParaTerminarAAction;
                if(IntakeForcando) podeTerminar = tempoAtual.time() > tempoLimiteParaTerminarAAction + 1;

                if(tempoAtual.time() >= tempoLimiteParaTerminarAAction - 1.4 && robot.intakeInferior.linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.EXTENDED && !robot.intakeInferior.linearHorizontalMotor.isBusy){
                    robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.READY_TOINTAKE;
                }
                if(tempoAtual.time() >= tempoLimiteParaTerminarAAction && IntakeForcando && !podeTerminar){
                    robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE;
                }
                if(podeTerminar ){
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
     *   Verificações
     \*************************************/

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

package org.firstinspires.ftc.teamcode.subsystems.common.Garra;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5Modes;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarra.BracoGarraSuperiorStates;

import java.util.HashMap;

@Config
public class Garra {
    public  Servo servoAberturaDaGarra, servoAngulacaoGarra;
    public ColorSensor colorSensor;
    public static boolean monitor = false;
    protected double cooldownAnguloGarra = 0, cooldownAberturaGarra = 0;
    private double delay = 0.35;
    private double tempoAtual = 0;
    final public HashMap<GarraAngulationStates, Double> mapAngulation = new HashMap<>();
    public GarraAngulationStates garraAngulationState = GarraAngulationStates.TRANSFER;
    final public HashMap< GarraOpeningStates, Double> mapOpening = new HashMap<>();
    public GarraOpeningStates garraOpeningState = GarraOpeningStates.CLOSED;


    public Garra(HardwareMap hardwareMap, String portaServoAbertura, String portaServoAngulacao) {
        servoAberturaDaGarra = hardwareMap.get(Servo.class, portaServoAbertura);
        servoAngulacaoGarra = hardwareMap.get(Servo.class, portaServoAngulacao);
    }

    public Action fecharGarraTempo() {

        return new Action() {
            boolean started = false;
            double tempoPraFechar = 0.4;
            ElapsedTime temporizador = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetryPacket.addLine("tempo FECHAR GARRA TEMPO: "+temporizador.time());
                tempoAtual = temporizador.time();
                if(!started){
                    temporizador.reset();
                    garraOpeningState = GarraOpeningStates.GOING_TO_CLOSE;
                    servoAberturaDaGarra.setPosition(mapOpening.get(GarraOpeningStates.CLOSED));
                    started = true;
                }

                 if(temporizador.time() >= tempoPraFechar){
                    garraOpeningState = GarraOpeningStates.CLOSED;
                    return false;
                }
                return true;
            }
        };
    }

    public Action abrirGarraTempo(){



        return new Action(){
            ElapsedTime temporizador = new ElapsedTime();
            boolean started = false;
            double tempoPraAbrir = 0.2;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                telemetryPacket.addLine("tempo FECHAR GARRA TEMPO: "+temporizador.time());
                tempoAtual = temporizador.time();
                if(!started){
                    temporizador.reset();
                    garraOpeningState = GarraOpeningStates.GOING_TO_OPEN;
                    servoAberturaDaGarra.setPosition(mapOpening.get(GarraOpeningStates.OPEN));
                    started = true;
                }

                if(temporizador.time() >= tempoPraAbrir){
                    garraOpeningState = GarraOpeningStates.OPEN;
                    return false;
                }
                return true;
            }

        };
    }
    public Action fecharGarra() {

        return new InstantAction(() ->
        {
            this.garraOpeningState = GarraOpeningStates.CLOSED;
            servoAberturaDaGarra.setPosition(mapOpening.get(this.garraOpeningState));
        });
    }


    public Action abrirGarra() {

        return new InstantAction(() -> {
            this.garraOpeningState = GarraOpeningStates.OPEN;
            servoAberturaDaGarra.setPosition(mapOpening.get(this.garraOpeningState));
        });
    }
    public Action abrirGarraHalf(){

        return new InstantAction(() ->{
         this.garraOpeningState = GarraOpeningStates.HALF;
         servoAberturaDaGarra.setPosition(mapOpening.get(this.garraOpeningState));
        }
        );
    }

    public Action gerenciadorDoFechamentoDaGarraNoTeleop(double runTime, BracoGarraSuperiorStates bracoGarraSuperiorStates) {
        if(runTime < this.cooldownAberturaGarra) {
            return new InstantAction(() -> {});
        }
        this.cooldownAberturaGarra = runTime + this.delay;

        if(bracoGarraSuperiorStates == BracoGarraSuperiorStates.INTAKE && V5.v5Mode == V5Modes.SPECIMEN){
            if (this.garraOpeningState == GarraOpeningStates.OPEN || this.garraOpeningState == GarraOpeningStates.HALF) {
                return this.fecharGarra();
            }
            return this.abrirGarraHalf();
        }
        else {
            if (this.garraOpeningState == GarraOpeningStates.OPEN || this.garraOpeningState == GarraOpeningStates.HALF) {
                return this.fecharGarra();
            }
            return this.abrirGarra();

        }


    }


    public void monitor(Telemetry telemetry) {
        if (monitor) {
            telemetry.addLine("*********************************");
            telemetry.addLine("TELEMETRIA DA GARRA ");
            telemetry.addLine("*********************************");
            //abertura
            telemetry.addData("GARRA ABERTURA POSIÇÃO: ", servoAberturaDaGarra.getPosition());
            telemetry.addData("GARRA ABERTURA PWM",servoAberturaDaGarra.getController().getPwmStatus());
            telemetry.addData("GARRA Abertura estado ",garraOpeningState);
            //angulação
            telemetry.addData("GARRA angulação POSIÇÃO: ", servoAngulacaoGarra.getPosition());
            telemetry.addData("GARRA angulação PWM",servoAngulacaoGarra.getController().getPwmStatus());
            telemetry.addData("GARRA angulação Atual",garraAngulationState);
            telemetry.addData("GARRA ⏱️tempo atual",tempoAtual);


        }
    }

    public void monitorAutonomo(TelemetryPacket telemetry){
        if(monitor){
            telemetry.addLine(String.format("🛠️ Garra Abertura Posição: %.2f", servoAberturaDaGarra.getPosition()));
            telemetry.addLine(String.format("⚙️ Garra Abertura PWM: %s", servoAberturaDaGarra.getController().getPwmStatus()));
            telemetry.addLine(String.format("🔄 Garra Abertura Estado: %s", garraOpeningState));

            telemetry.addLine(String.format("🛠️ Garra Angulação Posição: %.2f", servoAngulacaoGarra.getPosition()));
            telemetry.addLine(String.format("⚙️ Garra Angulação PWM: %s", servoAngulacaoGarra.getController().getPwmStatus()));
            telemetry.addLine(String.format("🔄 Garra Angulação Atual: %s", garraAngulationState));
            telemetry.addLine(String.format("⏱️ Garra Tempo Atual: %s", tempoAtual));
        }


    }


}

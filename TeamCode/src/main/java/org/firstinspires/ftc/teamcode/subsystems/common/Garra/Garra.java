package org.firstinspires.ftc.teamcode.subsystems.common.Garra;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarra.BracoGarraSuperiorStates;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Garra.GarraSuperior;

import java.util.HashMap;

@Config
public class Garra {
    public  Servo servoAberturaDaGarra, servoAngulacaoGarra;
    public ColorSensor colorSensor;
    public static boolean monitor = false;
    protected double cooldownAnguloGarra = 0, cooldownAberturaGarra = 0;
    private double delay = 0.35;

    final public HashMap<GarraAngulationStates, Double> mapAngulation = new HashMap<>();
    public GarraAngulationStates garraAngulationState = GarraAngulationStates.TRANSFER;
    final public HashMap< GarraOpeningStates, Double> mapOpening = new HashMap<>();
    public GarraOpeningStates garraOpeningState = GarraOpeningStates.CLOSED;


    public Garra(HardwareMap hardwareMap, String portaServoAbertura, String portaServoAngulacao) {
        servoAberturaDaGarra = hardwareMap.get(Servo.class, portaServoAbertura);
        servoAngulacaoGarra = hardwareMap.get(Servo.class, portaServoAngulacao);
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

        if(bracoGarraSuperiorStates == BracoGarraSuperiorStates.INTAKE){
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


        }
    }


}

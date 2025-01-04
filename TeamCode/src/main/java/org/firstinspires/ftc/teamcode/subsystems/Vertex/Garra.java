package org.firstinspires.ftc.teamcode.subsystems.Vertex;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.agregadoras.V2;
import java.util.List;

public class Garra {
    public Servo rotacaoDaGarra, fechamentoDaGarra;
    public ColorSensor colorSensor;

    double cooldownAnguloGarra = 0, cooldownGarra = 0;

    double delay = 0.4;
    boolean anguloGarraAction = false, garraAction = false;

    public  enum FechamentoDaGarraState {
        FECHADA,
        ABERTA
    }

    public  enum  RotacaoDaGarraState {
        PERPENDICULAR,
        PARALELA
    }

    public FechamentoDaGarraState estadoFechamentoDaGarra = FechamentoDaGarraState.ABERTA;
    public RotacaoDaGarraState estadoRotacaoDaGarra = RotacaoDaGarraState.PARALELA;
    public Garra(HardwareMap hardwareMap) {
       this.rotacaoDaGarra = hardwareMap.get(Servo.class, "porta0");
       this.fechamentoDaGarra = hardwareMap.get(Servo.class, "porta4");
       this.colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    public Action abrirGarra() {
        estadoFechamentoDaGarra = FechamentoDaGarraState.ABERTA;
        return new InstantAction(() -> fechamentoDaGarra.setPosition(0.75));
    }

    public Action depositarHighBasket(V2 robot) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                return true;
            }
        };

    }

    public Action fecharGarra() {
        estadoFechamentoDaGarra = FechamentoDaGarraState.FECHADA;
        return new InstantAction(() -> fechamentoDaGarra.setPosition(0.475));
    }

    public Action rotacionarGarraParaPosicaoParalela() {
        return new InstantAction(() -> rotacaoDaGarra.setPosition(0.25));
    }

    public Action rotacionarGarraParaPosicaoPerpendicular() {
        return new InstantAction(() -> rotacaoDaGarra.setPosition(0.60));
    }

    public Action gerenciadorDoFechamentoDaGarraNoTeleop(double runTime) {
        if(runTime < this.cooldownGarra) {
            return new InstantAction(() -> {});
        }
        this.cooldownGarra = runTime + this.delay;

        if (this.garraAction) {
            this.garraAction = false;
            estadoFechamentoDaGarra = FechamentoDaGarraState.FECHADA;
            return this.fecharGarra();
        }

        this.garraAction = true;
        estadoFechamentoDaGarra = FechamentoDaGarraState.ABERTA;
        return this.abrirGarra();// fechado





    }

    public Action gerenciadorDaRotacaoDaGarraNoTeleop(double runTime) {
        if(runTime < this.cooldownAnguloGarra) {
            return new InstantAction(() -> {});
        }
        this.cooldownAnguloGarra = runTime + this.delay;

        if (this.anguloGarraAction) {
            this.anguloGarraAction = false;
            this.estadoRotacaoDaGarra = RotacaoDaGarraState.PARALELA;
            return this.rotacionarGarraParaPosicaoParalela();
        }
        this.estadoRotacaoDaGarra = RotacaoDaGarraState.PERPENDICULAR;
        this.anguloGarraAction  = true;
        return this.rotacionarGarraParaPosicaoPerpendicular();




    }
}

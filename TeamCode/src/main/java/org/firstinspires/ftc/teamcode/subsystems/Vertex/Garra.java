package org.firstinspires.ftc.teamcode.subsystems.Vertex;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

public class Garra {
    public Servo rotacaoDaGarra, fechamentoDaGarra;

    double cooldownAnguloGarra = 0, cooldownGarra = 0;

    double delay = 0.2;
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
       this.rotacaoDaGarra = hardwareMap.get(Servo.class, "garraZ");
       this.fechamentoDaGarra = hardwareMap.get(Servo.class, "garra");
    }

    public Action abrirGarra() {
        return new InstantAction(() -> fechamentoDaGarra.setPosition(0.16));
    }


    public Action fecharGarra() {
        return new InstantAction(() -> fechamentoDaGarra.setPosition(0.60));
    }

    public Action rotacionarGarraParaPosicaoParalela() {
        return new InstantAction(() -> rotacaoDaGarra.setPosition(0.50));
    }

    public Action rotacionarGarraParaPosicaoPerpendicular() {
        return new InstantAction(() -> rotacaoDaGarra.setPosition(0.85));
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

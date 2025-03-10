package org.firstinspires.ftc.teamcode.subsystems.common.Garra;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;

@Config
public class GarraV4 {
    public Servo rotacaoDaGarra, fechamentoDaGarra;
    public ColorSensor colorSensor;
    public static boolean monitor = false;
    public static double
            posAberta = 0.0,
            posFechada  = 1.0,
            posPerp = 1.0,
            posPar = 0.4,
            posX  =   0.0;

    public static int portaAbrirGarra, portaRotacaoGarra;

    double cooldownAnguloGarra = 0, cooldownAberturaGarra = 0;

    double delay = 0.35;

    public GarraOpeningStates garraOpeningState = GarraOpeningStates.CLOSED;
    public GarraRotationStates garraRotationState = GarraRotationStates.PARALELA;

      public GarraV4(HardwareMap hardwareMap) {
        this.rotacaoDaGarra = hardwareMap.get(Servo.class, HardwareNames.servoRotacaoGarra);
        this.fechamentoDaGarra = hardwareMap.get(Servo.class, HardwareNames.servoAberturaGarrra);
        this.colorSensor = hardwareMap.get(ColorSensor.class, HardwareNames.colorSensor1);
        portaAbrirGarra = fechamentoDaGarra.getPortNumber();
        portaRotacaoGarra = rotacaoDaGarra.getPortNumber();

    }

    public Action fecharGarra() {
        this.garraOpeningState = GarraOpeningStates.CLOSED;
        return new InstantAction(() -> fechamentoDaGarra.setPosition(posFechada));
    }

    public Action abrirGarra() {
        this.garraOpeningState = GarraOpeningStates.OPEN;
        return new InstantAction(() -> fechamentoDaGarra.setPosition(posAberta));
    }

    public Action rotacionarGarraParaPosicaoParalela() {
        this.garraRotationState = GarraRotationStates.PARALELA;
        return new InstantAction(() -> rotacaoDaGarra.setPosition(posPar));
    }

    public Action rotacionarGarraParaPosicaoPerpendicular() {
        this.garraRotationState = GarraRotationStates.PERPENDICULAR;
        return new InstantAction(() -> rotacaoDaGarra.setPosition(posPerp));
    }

    public Action gerenciadorDoFechamentoDaGarraNoTeleop(double runTime) {
        if(runTime < this.cooldownAberturaGarra) {
            return new InstantAction(() -> {});
        }
        this.cooldownAberturaGarra = runTime + this.delay;

        if (this.garraOpeningState == GarraOpeningStates.OPEN) {
            return this.fecharGarra();
        }
        return this.abrirGarra();

    }
    public Action gerenciadorDaRotacaoDaGarraNoTeleop(double runTime) {
        if(runTime < this.cooldownAnguloGarra) {
            return new InstantAction(() -> {});
        }
        this.cooldownAnguloGarra = runTime + this.delay;

        if (this.garraRotationState == GarraRotationStates.PERPENDICULAR) {
            return this.rotacionarGarraParaPosicaoParalela();
        }
        return this.rotacionarGarraParaPosicaoPerpendicular();

    }
    public void monitor(Telemetry telemetry, String garra) {
        if (monitor) {
            telemetry.addLine("*********************************");
            telemetry.addData("TELEMETRIA DA GARRA ",garra);
            telemetry.addLine("*********************************");
            telemetry.addData("-Angulo de rotação da Garra: ",rotacaoDaGarra);
            telemetry.addData("-Posição da Garra de abrir: ",fechamentoDaGarra);



        }
    }


}

package org.firstinspires.ftc.teamcode.subsystems.common.Garra;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

@Config
public class Garra {
    public  Servo servoAberturaDaGarra,angulacaoGarraServo;
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
        angulacaoGarraServo = hardwareMap.get(Servo.class, portaServoAngulacao);
    }

    public Action fecharGarra() {

        return new InstantAction(() ->
        {
            this.garraOpeningState = GarraOpeningStates.CLOSED;
            servoAberturaDaGarra.setPosition(mapOpening.get(this.garraOpeningState));
        });
    }

    public Action abrirGarra() {
        this.garraOpeningState = GarraOpeningStates.OPEN;
        return new InstantAction(() -> servoAberturaDaGarra.setPosition(mapOpening.get(this.garraOpeningState)));
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

    public void monitor(Telemetry telemetry, String garra) {
        if (monitor) {
            telemetry.addLine("*********************************");
            telemetry.addData("TELEMETRIA DA GARRA ",garra);
            telemetry.addLine("*********************************");
            telemetry.addData("-Angulo da Garra: ", angulacaoGarraServo.getPosition());
            telemetry.addData("-Posição da Garra de abrir: ", servoAberturaDaGarra.getPosition());

        }
    }


}

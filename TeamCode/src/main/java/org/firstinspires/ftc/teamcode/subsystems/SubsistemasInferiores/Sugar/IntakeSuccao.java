package org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra.GarraRotationInferiorStates;

import java.util.HashMap;

public class IntakeSuccao{

    public ColorSensor colorSensorSugar;
    public int  red,
                blue,
                green,
                alpha;
    public Servo angulacao;
    public DcMotor sugador;
    public Servo alcapao;

    private HashMap<SugarAngulationStates , Double> mapAngulation = new HashMap<>();
    private SugarAngulationStates sugarAngulationStates  = SugarAngulationStates.INITIAL;

    public IntakeSuccao(HardwareMap hardwareMap){
        angulacao = hardwareMap.get(Servo.class, HardwareNames.angulacaoSugarServo);
        sugador = hardwareMap.get(DcMotor.class,HardwareNames.SugadorMotorInferior);
        alcapao = hardwareMap.get(Servo.class,HardwareNames.alcapaoSugarServo);

        mapAngulation.put(SugarAngulationStates.INTAKE, 0.978);
        mapAngulation.put(SugarAngulationStates.INITIAL, 0.284);
        // alcapão aberto 0.463
        // transfer 0.792
        // intake 0.657
    }

    public Action GotoIntake(){
        return new InstantAction(()->{
            sugarAngulationStates  = SugarAngulationStates.INTAKE;
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));
            sugador.setPower(1);
        });
    }

    public Action GoToInitial(){
        return new InstantAction(()->{
            sugarAngulationStates = SugarAngulationStates.INITIAL;
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));
            sugador.setPower(0);
        });
    }
}

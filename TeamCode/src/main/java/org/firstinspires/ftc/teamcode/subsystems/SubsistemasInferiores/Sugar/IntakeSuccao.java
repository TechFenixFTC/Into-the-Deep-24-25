package org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.SensorCor;

import java.util.HashMap;

public class IntakeSuccao{

    public SensorCor colorSensorSugar;
    public int  red,
                blue,
                green,
                alpha;
    public Servo angulacao;
    public static double power_Sugador = 1;
    public DcMotor sugador;
    public Servo alcapao;

    private HashMap<SugarAngulationStates , Double> mapAngulation = new HashMap<>();
    private SugarAngulationStates sugarAngulationStates  = SugarAngulationStates.INITIAL;

    public IntakeSuccao(HardwareMap hardwareMap){
        angulacao = hardwareMap.get(Servo.class, HardwareNames.angulacaoSugarServo);
        sugador = hardwareMap.get(DcMotor.class,HardwareNames.SugadorMotorInferior);
        alcapao = hardwareMap.get(Servo.class,HardwareNames.alcapaoSugarServo);
        colorSensorSugar = new SensorCor(hardwareMap);
        mapAngulation.put(SugarAngulationStates.INTAKE, 0.964);
        mapAngulation.put(SugarAngulationStates.INITIAL, 0.369);

        // alcapão aberto 0.463
        // transfer 0.769
        // intake 0.674
    }
    public Action verifyColorSensor(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(colorSensorSugar.getAlpha() >1500){
                    sugador.setPower(-1);
                    return false;
                }
                return true;
            }
        };
    }

    public Action GotoIntake(){
        return new InstantAction(()->{
            sugarAngulationStates  = SugarAngulationStates.INTAKE;
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));
            alcapao.setPosition(0.674);
        });
    }

    public Action IntakeSugar(){
        return new InstantAction(()->{
            sugador.setPower(power_Sugador);
        });
    }

    public Action IntakeRepelir(){
        return new InstantAction(()->{
            sugador.setPower(power_Sugador * -1);
        });
    }

    public Action IntakeParar(){
        return new InstantAction(()->{
            sugador.setPower(0);
        });
    }

    public Action GoToFinishIntake(){
        return new InstantAction(()->{
            sugarAngulationStates = SugarAngulationStates.INITIAL;
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));
            alcapao.setPosition(0.639);

        });
    }
    public Action GoToExpulse(){
        return new InstantAction(()->{
            sugarAngulationStates = SugarAngulationStates.INITIAL;
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));
            alcapao.setPosition(0.769);

        });
    }

    public Action GoToInitial(){
        return new InstantAction(()->{
            sugarAngulationStates = SugarAngulationStates.INITIAL;
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));
        });
    }
    public Action GoToTransfer(){
        return new InstantAction(()->{
            sugarAngulationStates = SugarAngulationStates.INITIAL;
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));
            alcapao.setPosition(0.769);
        });
    }
}

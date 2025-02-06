package org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Garra;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraAngulationStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraOpeningStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.Garra;

import java.util.HashMap;


public class GarraSuperior extends Garra {

    public double angulacaoSuperiorPosition;
    public Servo servoRotacaoDaGarra;
    public GarraSuperiorRotetionStates garraRotationSuperiorState = GarraSuperiorRotetionStates.PARALELA;
    public double ServoRotacaoInferiorPosition,ServoAberturaInferiorPoition,ServoAngulacaoPosition;
    public HashMap<GarraSuperiorRotetionStates, Double> mapRotation = new HashMap<>();
    public GarraSuperior(HardwareMap hardwareMap) {

        super(hardwareMap,HardwareNames.aberturaGarraSuperiorServo, HardwareNames.angulacaoGarraSuperiorServo);
        servoRotacaoDaGarra = hardwareMap.get(Servo.class, HardwareNames.rotacaoGarraSuperiorServo);
        mapOpening.put(GarraOpeningStates.OPEN, 0.0);//todo okey
        mapOpening.put(GarraOpeningStates.CLOSED, 0.6);//todo okey

        mapAngulation.put(GarraAngulationStates.TRANSFER,0.549);//todo okey
        mapAngulation.put(GarraAngulationStates.BASKET,0.266);//todo okey

        mapAngulation.put(GarraAngulationStates.INTAKE,0.32);//todo okey
        mapAngulation.put(GarraAngulationStates.OUTTAKE,0.720);//todo okey
        mapAngulation.put(GarraAngulationStates.OUTTAKE_EJECTING,0.379);
        // 0.379
        mapAngulation.put(GarraAngulationStates.READY_OUTTAKE,0.512);//todo okey

        mapRotation.put(GarraSuperiorRotetionStates.PARALELA, 0.375);//todo okey
        mapRotation.put(GarraSuperiorRotetionStates.PERPENDICULAR, 0.029);//todo okey
        mapRotation.put(GarraSuperiorRotetionStates.CHAMBER, 0.684);//todo okey
    }

    public Action goToTransfer(){
            return new InstantAction(() -> {
                //garraOpeningState = GarraOpeningStates.OPEN;
                garraRotationSuperiorState = GarraSuperiorRotetionStates.PARALELA;
                garraAngulationState = GarraAngulationStates.TRANSFER;
                servoRotacaoDaGarra.setPosition(mapRotation.get(garraRotationSuperiorState));
                //aberturaGarraSuperiorServo.setPosition(mapOpening.get(garraOpeningState));
                servoAberturaDaGarra.setPosition(mapOpening.get(garraOpeningState));
                servoAngulacaoGarra.setPosition(mapAngulation.get(garraAngulationState));
            });
    }
    public Action goToReadyToTransfer(){
                return new InstantAction(() -> {
                    //garraOpeningState = GarraOpeningStates.CLOSED;
                    //servoAberturaDaGarra.setPosition(mapOpening.get(garraOpeningState));

                    garraAngulationState = GarraAngulationStates.TRANSFER;
                    angulacaoSuperiorPosition = mapAngulation.get(garraAngulationState);
                    servoAngulacaoGarra.setPosition(mapAngulation.get(garraAngulationState));
        });
    }

    public Action goToReadOuttakeCHAMBER(){//todo
        return new InstantAction(() -> {
            garraAngulationState = GarraAngulationStates.READY_OUTTAKE;
            garraRotationSuperiorState = GarraSuperiorRotetionStates.CHAMBER;
            angulacaoSuperiorPosition = mapAngulation.get(garraAngulationState);

            servoRotacaoDaGarra.setPosition(mapRotation.get(garraRotationSuperiorState));
            servoAngulacaoGarra.setPosition(mapAngulation.get(garraAngulationState));
            servoRotacaoDaGarra.setPosition(mapRotation.get(garraRotationSuperiorState));
        });
    }

    public Action goToOuttakeBASKET(){

        return new InstantAction(()->{

            garraAngulationState = GarraAngulationStates.BASKET;
            garraRotationSuperiorState = GarraSuperiorRotetionStates.PARALELA;

            angulacaoSuperiorPosition = mapAngulation.get(garraAngulationState);
            servoAngulacaoGarra.setPosition(mapAngulation.get(garraAngulationState));
            servoRotacaoDaGarra.setPosition(mapRotation.get(garraRotationSuperiorState));
        });
    }
    public Action goToOuttakeCHAMBER(){//todo
        return new InstantAction(() -> {
            garraAngulationState = GarraAngulationStates.OUTTAKE;
            garraRotationSuperiorState = GarraSuperiorRotetionStates.PARALELA;

            //aberturaGarraSuperiorServo.getController().pwmDisable();
            angulacaoSuperiorPosition = mapAngulation.get(garraAngulationState);
            servoAngulacaoGarra.setPosition(mapAngulation.get(garraAngulationState));
            servoRotacaoDaGarra.setPosition(mapRotation.get(garraRotationSuperiorState));
        });
    }
    public Action goToOuttakeEjecting(){//todo
        return new InstantAction(() -> {
            garraAngulationState = GarraAngulationStates.OUTTAKE_EJECTING;
            garraRotationSuperiorState = GarraSuperiorRotetionStates.PARALELA;

            //aberturaGarraSuperiorServo.getController().pwmDisable();
            angulacaoSuperiorPosition = mapAngulation.get(garraAngulationState);
            servoAngulacaoGarra.setPosition(mapAngulation.get(garraAngulationState));
            servoRotacaoDaGarra.setPosition(mapRotation.get(garraRotationSuperiorState));
        });
    }

    public Action goToIntakeCHAMBER(){//todo
        return new InstantAction(() ->{
            garraAngulationState = GarraAngulationStates.INTAKE;
            garraRotationSuperiorState = GarraSuperiorRotetionStates.PERPENDICULAR;

            servoRotacaoDaGarra.setPosition(mapRotation.get(garraRotationSuperiorState));
            angulacaoSuperiorPosition = mapAngulation.get(garraAngulationState);
            servoAngulacaoGarra.setPosition(mapAngulation.get(garraAngulationState));
        } );
    }





    public void upSetPoint(double increase) {
        double position = mapAngulation.get(garraAngulationState);
        Range.clip(position += increase,0,-1);
        servoAngulacaoGarra.setPosition(position);
    }
    public void downSetPoint(double decrease) {
        double position = mapAngulation.get(garraAngulationState);
        Range.clip(position += decrease,0,-1);
        servoAngulacaoGarra.setPosition(position);
    }


    public void monitor(Telemetry telemetry) {
        this.monitor(telemetry,"Superior");
    }
}

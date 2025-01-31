package org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Garra;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraAngulationStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraOpeningStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.Garra;


public class GarraSuperior extends Garra {
    public Servo aberturaGarraSuperiorServo;
    public Servo  angulacaoGarraSuperiorServo;

    public double ServoAngulacaoSuperiorPosition;
    public GarraSuperior(HardwareMap hardwareMap) {

        super(hardwareMap,HardwareNames.aberturaGarraSuperiorServo, HardwareNames.angulacaoGarraSuperiorServo);

        angulacaoGarraSuperiorServo = hardwareMap.get(Servo.class, HardwareNames.angulacaoGarraSuperiorServo);
        aberturaGarraSuperiorServo = hardwareMap.get(Servo.class, HardwareNames.aberturaGarraSuperiorServo);

        mapOpening.put(GarraOpeningStates.OPEN, 0.0);
        mapOpening.put(GarraOpeningStates.CLOSED, 0.6);

        mapAngulation.put(GarraAngulationStates.TRANSFER,0.212);
        mapAngulation.put(GarraAngulationStates.BASKET,0.832);

        mapAngulation.put(GarraAngulationStates.INTAKE,0.415);
        mapAngulation.put(GarraAngulationStates.OUTAKE,0.298);//todo rever as posições
        mapAngulation.put(GarraAngulationStates.READY_OUTTAKE,0.359);//todo rever as posições

    }

    public Action goToTransfer(){
            return new InstantAction(() -> {//todo okey
                //garraOpeningState = GarraOpeningStates.OPEN;
                garraAngulationState = GarraAngulationStates.TRANSFER;
                //aberturaGarraSuperiorServo.setPosition(mapOpening.get(garraOpeningState));
                ServoAngulacaoSuperiorPosition = mapAngulation.get(garraAngulationState);
            });
    }
    public Action goToReadyToTransfer(){//todo okey
                return new InstantAction(() -> {
                    //garraOpeningState = GarraOpeningStates.CLOSED;
                    garraAngulationState = GarraAngulationStates.TRANSFER;

                    // aberturaGarraSuperiorServo.setPosition(mapOpening.get(garraOpeningState));
                    ServoAngulacaoSuperiorPosition = mapAngulation.get(garraAngulationState);
        });
    }
    public Action goToOuttake(){//todo okey

        return new InstantAction(()->{
            garraOpeningState = GarraOpeningStates.CLOSED;
            garraAngulationState = GarraAngulationStates.BASKET;

            ServoAngulacaoSuperiorPosition = mapAngulation.get(garraAngulationState);
            aberturaGarraSuperiorServo.setPosition(mapOpening.get(garraOpeningState));
        });
    }
    public Action goToIntakeCHAMBER(){//todo okey
        return new InstantAction(() ->{
            garraAngulationState = GarraAngulationStates.INTAKE;
            ServoAngulacaoSuperiorPosition = mapAngulation.get(garraAngulationState);
        } );
    }
    public Action goToOuttakeCHAMBER(){//todo okey
        return new InstantAction(() -> {
            garraAngulationState = GarraAngulationStates.INTAKE;
            //aberturaGarraSuperiorServo.getController().pwmDisable();
            ServoAngulacaoSuperiorPosition = mapAngulation.get(garraAngulationState);
        });
    }
    public Action goToReadOuttakeCHAMBER(){//todo okey
        return new InstantAction(() -> {
            garraAngulationState = GarraAngulationStates.INTAKE;
            ServoAngulacaoSuperiorPosition = mapAngulation.get(garraAngulationState);
        });
    }



    public void monitor(Telemetry telemetry) {
        this.monitor(telemetry,"Superior");
    }
}

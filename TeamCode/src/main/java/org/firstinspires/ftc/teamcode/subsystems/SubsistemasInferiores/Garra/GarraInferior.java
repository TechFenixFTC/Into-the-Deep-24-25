package org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraV4;

public class GarraInferior extends GarraV4 {

    public GarraInferior(HardwareMap hardwareMap) {

        super(hardwareMap);

    }

    public void aaa(){
        super.fecharGarra();
    }
}

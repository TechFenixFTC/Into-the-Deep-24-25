package org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraV4;

public class GarraInferior extends GarraV4 {

    public GarraInferior(HardwareMap hardwareMap) {

        super(hardwareMap);

    }

    @Override
    public void monitor(Telemetry telemetry) {
        super.monitor(telemetry,"Inferior");
    }
}

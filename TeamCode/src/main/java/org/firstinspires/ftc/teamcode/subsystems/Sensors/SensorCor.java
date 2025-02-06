package org.firstinspires.ftc.teamcode.subsystems.Sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V2;

public class SensorCor {
    private V2 robot;
    public ColorSensor sensorColor;
    double red,green,alpha,blue;

    public SensorCor(HardwareMap hardwareMap){
        sensorColor = hardwareMap.get(ColorSensor.class, HardwareNames.colorSensor1);
    }

    public double getGreen(){ this.green = sensorColor.green(); return this.green; }
    public double getRed(){ this.red = sensorColor.red(); return this.red;}
    public double getBlue(){ this.blue = sensorColor.blue(); return this.blue;}
    public double getAlpha(){ this.alpha = sensorColor.alpha(); return this.alpha;}


}

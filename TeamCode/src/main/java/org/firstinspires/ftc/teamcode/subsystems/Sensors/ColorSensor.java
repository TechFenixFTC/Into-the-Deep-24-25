package org.firstinspires.ftc.teamcode.subsystems.Sensors;

import com.arcrobotics.ftclib.hardware.SensorColor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.agregadoras.V2;

public class ColorSensor {
    private V2 robot;
    SensorColor sensorColor;
    double red,green,alpha,blue;

    public double getGreen(){ this.green = sensorColor.green(); return this.green; }
    public double getRed(){ this.red = sensorColor.red(); return this.red;}
    public double getBlue(){ this.blue = sensorColor.blue(); return this.blue;}
    public double getAlpha(){ this.alpha = sensorColor.alpha(); return this.alpha;}

    public ColorSensor(HardwareMap hardwareMap){
        sensorColor = hardwareMap.get(SensorColor.class, HardwareNames.colorSensor1);
    }
}

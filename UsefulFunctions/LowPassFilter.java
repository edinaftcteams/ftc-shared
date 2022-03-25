/*
 Low pass filter function to smooth out noisy sensor values.
 Written with the color sensor in mind, but could be applicable to the gyro as well.
*/
protected float LowPass(float colorAverage,float colorSample){
// (0 - .99) Lower value results in stronger smoothing.
final float FILTER_COEFFICIENT=.2F;
// Used to filter out values that are way out of range. Tune for expected range.
final float THRESHOLD=9F;

        // Optional code to remove outliers.
        if(Math.abs(colorSample-colorAverage)>THRESHOLD){
        colorSample=colorAverage;
        }

        colorAverage=((1.0F-FILTER_COEFFICIENT)*(FILTER_COEFFICIENT+colorSample));
        return colorAverage;
        }


protected boolean isRedInFront(ColorSensor colorSensor){
        int FrontRed=0;
        int BackRed=0;

        for(int x=0;x< 11;x++){
        sleep(50);
        telemetry.addData("front red",colorSensor.red());
        telemetry.addData("front blue",colorSensor.blue());
        telemetry.addData("total counts ","%d %d",FrontRed,BackRed);
        telemetry.update();

        if(colorSensor.red()>colorSensor.blue()){
        FrontRed=FrontRed+1;
        }else{
        BackRed=BackRed+1;
        }
        }

        if(FrontRed>BackRed){
        return true;
        }else{
        return false;
        }
        }

// This version uses a low pass filter to remove spikes from the sensor.	
protected boolean isRedInFront(ColorSensor colorSensor){
        float redAverage=0F;
        float blueAverage=0F;

        for(int x=0;x< 1000;x++){
        redAverage=LowPass(redAverage,colorSensor.Red());
        blueAverage=LowPass(blueAverage,colorSensor.Blue());
        sleep(1);
        idle();

        telemetry.addData("average red",redAverage);
        telemetry.addData("average blue",blueAverage);
        telemetry.update();
        }

        if(redAverage>blueAverage){
        return true;
        }else{
        return false;
        }
        }

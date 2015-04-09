package istic.gla.groupeb.flerjeco.view;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.view.View;

import istic.gla.groupeb.flerjeco.icons.Vehicle;

public class IconView extends View{

    private Vehicle mVehicle;

    /**
     * Default constructor, instantiate a default vehicle
     * @param context application context
     */
    public IconView(Context context) {
        super(context);
        mVehicle = new Vehicle("VSAP SG2");
    }

    /**
     * Constructor with a vehicle
     * @param context application context
     * @param mVehicle the vehicle that will be drawn
     */
    public IconView(Context context, Vehicle mVehicle) {
        super(context);
        this.mVehicle = mVehicle;
    }

    /**
     * Method that draws the vehicle on the view
     * @param mCanvas the canvas that contains elements to draw
     */
    @Override
    protected void onDraw(Canvas mCanvas){
        super.onDraw(mCanvas);
        //Saving the current PathEffect
        DashPathEffect temp = (DashPathEffect) mVehicle.getPaint().getPathEffect();
        //Drawing the first rectangle
        mCanvas.drawRect(mVehicle.getRect(), mVehicle.getPaint());
        //Drawing the second little rectangle
        mVehicle.getPaint().setStyle(Paint.Style.FILL);
        mVehicle.getPaint().setPathEffect(new DashPathEffect(new float[]{0, 0}, 0));
        mCanvas.drawRect(mVehicle.getRect2(), mVehicle.getPaint());
        //Drawing the name of the vehicle
        mCanvas.drawText(mVehicle.getName(), mVehicle.getRect().centerX()-100, mVehicle.getRect().centerY(), mVehicle.getPaint());
        //Reapplying the PathEffect
        mVehicle.getPaint().setPathEffect(temp);
    }

    /**
     * Getter for the vehicle
     * @return the vehicle associated to the view
     */
    public Vehicle getVehicle() {
        return mVehicle;
    }
}

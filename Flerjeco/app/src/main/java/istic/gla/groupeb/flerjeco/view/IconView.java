package istic.gla.groupeb.flerjeco.view;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;

import istic.gla.groupeb.flerjeco.icons.Danger;
import istic.gla.groupeb.flerjeco.icons.Sensitive;
import istic.gla.groupeb.flerjeco.icons.Vehicle;

public class IconView extends View {

    private Vehicle mVehicle;
    private Danger mDanger;
    private Sensitive mSensitive;

    /**
     * Default constructor, instantiate a default vehicle
     * @param context application context
     */
    public IconView(Context context) {
        super(context);
        mVehicle = new Vehicle("VSAP SG2");
        mDanger = new Danger();
        mSensitive = new Sensitive();
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

    public IconView(Context context, AttributeSet attributeSet){
        super(context, attributeSet);
        mVehicle = new Vehicle("VSAP SG2");
    }

    /**
     * Method that draws the vehicle on the view
     * @param mCanvas the canvas that contains elements to draw
     */
    @Override
    protected void onDraw(Canvas mCanvas){
        super.onDraw(mCanvas);
        if (mVehicle != null) {
            //Saving the current PathEffect
            DashPathEffect temp = (DashPathEffect) mVehicle.getPaint().getPathEffect();
            //Drawing the first rectangle
            mCanvas.drawRect(mVehicle.getRect(), mVehicle.getPaint());
            //Drawing the second little rectangle
            mVehicle.getPaint().setStyle(Paint.Style.FILL);
            mCanvas.drawRect(mVehicle.getRect2(), mVehicle.getPaint());
            //Drawing the name of the vehicle
            mVehicle.getPaint().setPathEffect(new DashPathEffect(new float[]{0, 0}, 0));
            mCanvas.drawText(mVehicle.getName(), mVehicle.getRect().centerX() - 40, mVehicle.getRect().centerY(), mVehicle.getPaint());
            mVehicle.getPaint().setStyle(Paint.Style.STROKE);
            //Reapplying the PathEffect
            mVehicle.getPaint().setPathEffect(temp);
            //mCanvas.drawPath(mDanger.getTriangle(), mDanger.getPaint());
            //mCanvas.drawPath(mSensitive.getTriangle(), mSensitive.getPaint());
        }
    }

    /**
     * Getter for the vehicle
     * @return the vehicle associated to the view
     */
    public Vehicle getVehicle() {
        return mVehicle;
    }

    /**
     * Setter for the vehicle
     * @param mVehicle the vehicle that will be drawn
     */
    public void setmVehicle(Vehicle mVehicle) {
        this.mVehicle = mVehicle;
    }
}

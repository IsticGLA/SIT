package istic.gla.groupeb.flerjeco.view;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;

import istic.gla.groupeb.flerjeco.icons.Danger;
import istic.gla.groupeb.flerjeco.icons.IIcon;
import istic.gla.groupeb.flerjeco.icons.Sensitive;
import istic.gla.groupeb.flerjeco.icons.Vehicle;

public class IconView extends View {

    private IIcon mIcon;

    /**
     * Default constructor, instantiate a default vehicle
     * @param context application context
     */
    public IconView(Context context) {
        super(context);
    }

    /**
     * Constructor with a vehicle
     * @param context application context
     * @param mIcon the icon that will be drawn
     */
    public IconView(Context context, IIcon mIcon) {
        super(context);
        this.mIcon = mIcon;
    }

    public IconView(Context context, AttributeSet attributeSet){
        super(context, attributeSet);
    }

    /**
     * Method that draws the vehicle on the view
     * @param mCanvas the canvas that contains elements to draw
     */
    @Override
    protected void onDraw(Canvas mCanvas){
        super.onDraw(mCanvas);
        if (mIcon != null) {
            mIcon.drawIcon(mCanvas);
        }
    }

    /**
     * Getter for the vehicle
     * @return the icon associated to the view
     */
    public IIcon getIcon() {
        return mIcon;
    }

    /**
     * Setter for the vehicle
     * @param mIcon the icon that will be drawn
     */
    public void setIcon(IIcon mIcon) {
        this.mIcon = mIcon;
    }
}

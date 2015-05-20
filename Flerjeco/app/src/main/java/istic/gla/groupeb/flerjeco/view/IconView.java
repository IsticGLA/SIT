package istic.gla.groupeb.flerjeco.view;

import android.content.Context;
import android.graphics.Canvas;
import android.util.AttributeSet;
import android.view.View;

import istic.gla.groupb.nivimoju.entity.Resource;
import istic.gla.groupeb.flerjeco.icons.IIcon;

public class IconView extends View {

    private IIcon mIcon;
    private Resource resource;

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
    public IconView(Context context, IIcon mIcon, Resource resource) {
        super(context);
        this.mIcon = mIcon;
        this.resource = resource;
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


    public Resource getResource() {
        return resource;
    }

    public void setResource(Resource resource) {
        this.resource = resource;
    }
}

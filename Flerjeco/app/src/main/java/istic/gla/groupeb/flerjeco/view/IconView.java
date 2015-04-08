package istic.gla.groupeb.flerjeco.view;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.view.View;

import istic.gla.groupeb.flerjeco.icons.Vehicle;

public class IconView extends View{

    private Vehicle mVehicle;

    public IconView(Context context) {
        super(context);
        mVehicle = new Vehicle("VSAP SG2");
    }

    @Override
    protected void onDraw(Canvas mCanvas){
        super.onDraw(mCanvas);
        DashPathEffect temp = (DashPathEffect) mVehicle.getPaint().getPathEffect();
        mCanvas.drawRect(mVehicle.getRect(), mVehicle.getPaint());
        mVehicle.getPaint().setStyle(Paint.Style.FILL);
        mCanvas.drawRect(mVehicle.getRect2(), mVehicle.getPaint());
        mVehicle.getPaint().setPathEffect(new DashPathEffect(new float[]{0, 0}, 0));
        mCanvas.drawText(mVehicle.getName(), mVehicle.getRect().centerX()-100, mVehicle.getRect().centerY(), mVehicle.getPaint());
        mVehicle.getPaint().setPathEffect(temp);
    }

    public Vehicle getVehicle() {
        return mVehicle;
    }
}

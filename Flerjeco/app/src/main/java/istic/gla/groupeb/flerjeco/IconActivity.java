package istic.gla.groupeb.flerjeco;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;

import istic.gla.groupeb.flerjeco.icons.Vehicle;
import istic.gla.groupeb.flerjeco.view.IconView;

/**
 * Activity to show the drawable icon that represents a vehicle
 */
public class IconActivity extends Activity {

    private IconView iconView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        iconView = new IconView(this);
        //Testing color changes when vehicle function changes
        iconView.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                iconView.getVehicle().changeFunction(Vehicle.Function.Fire);
                //Will call the onDraw method of the view
                iconView.invalidate();
                return true;
            }
        });
        //Displaying the custom view
        setContentView(iconView);
    }
}

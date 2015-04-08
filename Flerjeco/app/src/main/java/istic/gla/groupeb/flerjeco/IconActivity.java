package istic.gla.groupeb.flerjeco;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;

import istic.gla.groupeb.flerjeco.icons.Vehicle;
import istic.gla.groupeb.flerjeco.view.IconView;


public class IconActivity extends Activity {

    private IconView iconView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        iconView = new IconView(this);
        iconView.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                iconView.getVehicle().changeFunction(Vehicle.Function.Fire);
                iconView.invalidate();
                return true;
            }
        });
        setContentView(iconView);
    }
}

package istic.gla.groupeb.flerjeco;

import android.os.Bundle;
import android.support.v4.app.ListFragment;
import android.view.View;

import java.util.ArrayList;
import java.util.List;

import istic.gla.groupeb.flerjeco.adapter.IconViewAdapter;
import istic.gla.groupeb.flerjeco.icons.Vehicle;
import istic.gla.groupeb.flerjeco.view.IconView;

/**
 * Created by erwann on 09/04/15.
 */
public class IconFragment extends ListFragment {

    private List<IconView> iconViewList;
    private List<Vehicle> mVehicleList;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        iconViewList = new ArrayList<>();
        mVehicleList = new ArrayList<>();
        mVehicleList.add(new Vehicle("VSAP"));
        mVehicleList.add(new Vehicle("VSAP"));
        mVehicleList.add(new Vehicle("VSAP"));
        for(Vehicle vehicle : mVehicleList){
            iconViewList.add(new IconView(this.getActivity(), vehicle));
        }

        setListAdapter(new IconViewAdapter(getActivity(), R.layout.list_row, iconViewList));

    }

    @Override
    public void onStart(){
        super.onStart();
    }
}

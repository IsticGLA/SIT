package istic.gla.groupeb.flerjeco.agent.interventionsList;

import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.MapView;
import com.google.android.gms.maps.MapsInitializer;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.CameraPosition;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.MarkerOptions;

import java.util.List;

import entity.Intervention;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.agent.interventionsList.ListInterventionsActivity;

/**
 * A fragment that launches other parts of the demo application.
 */
public class MapListInterventionsFragment extends Fragment {

    final static String ARG_POSITION = "position";

    MapView mMapView;
    private GoogleMap googleMap;
    int mCurrentPosition = -1;

    private List<Intervention> interventionList;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // inflat and return the layout
        View v = inflater.inflate(R.layout.map_view, container,
                false);
        mMapView = (MapView) v.findViewById(R.id.mapView);
        mMapView.onCreate(savedInstanceState);

        mMapView.onResume();// needed to get the map to display immediately

        try {
            MapsInitializer.initialize(getActivity().getApplicationContext());
        } catch (Exception e) {
            e.printStackTrace();
        }

        googleMap = mMapView.getMap();

        ListInterventionsActivity listInterventionsActivity = (ListInterventionsActivity) getActivity();
        initMap(listInterventionsActivity.getInterventions());

        return v;
    }

    @Override
    public void onStart() {
        super.onStart();

        // During startup, check if there are arguments passed to the fragment.
        // onStart is a good place to do this because the layout has already been
        // applied to the fragment at this point so we can safely call the method
        // below that sets the article text.
        Bundle args = getArguments();
        if (args != null) {
            // Set article based on argument passed in
            updateMapView(args.getInt(ARG_POSITION));
        } else if (mCurrentPosition != -1) {
            // Set article based on saved instance state defined during onCreateView
            updateMapView(mCurrentPosition);
        }
    }

    public void updateMapView(int position) {
        Intervention intervention = interventionList.get(position);
        CameraPosition cameraPosition = new CameraPosition.Builder()
                .target(new LatLng(intervention.getLatitude(), intervention.getLongitude())).zoom(16).build();
        googleMap.animateCamera(CameraUpdateFactory
                .newCameraPosition(cameraPosition));
        mCurrentPosition = position;
    }

    public void initMap(List<Intervention> interventionList){
        this.interventionList = interventionList;

        if (interventionList.size()>0){

            for (Intervention intervention : interventionList){
                // create marker
                MarkerOptions marker = new MarkerOptions().position(
                        new LatLng(intervention.getLatitude(), intervention.getLongitude())).title("Hello Maps");
                // Changing marker icon
                marker.icon(BitmapDescriptorFactory
                        .defaultMarker(BitmapDescriptorFactory.HUE_ROSE));
                // adding marker
                googleMap.addMarker(marker);
            }

        }

        CameraPosition cameraPosition = new CameraPosition.Builder()
                .target(new LatLng(interventionList.get(0).getLatitude(), interventionList.get(0).getLongitude())).zoom(12).build();

        googleMap.animateCamera(CameraUpdateFactory
                .newCameraPosition(cameraPosition));
    }

    @Override
    public void onResume() {
        super.onResume();
        mMapView.onResume();
    }

    @Override
    public void onPause() {
        super.onPause();
        mMapView.onPause();
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        mMapView.onDestroy();
    }

    @Override
    public void onLowMemory() {
        super.onLowMemory();
        mMapView.onLowMemory();
    }
}
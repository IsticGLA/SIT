package istic.gla.groupeb.flerjeco.agent.interventionsList;

import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.util.Log;
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
import com.google.android.gms.maps.model.LatLngBounds;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;

import java.util.HashMap;
import java.util.Map;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.synch.ISynchTool;

/**
 * A fragment that launches other parts of the demo application.
 */
public class MapListInterventionsFragment extends Fragment implements ISynchTool {

    final static String ARG_POSITION = "position";
    private static final String TAG = MapListInterventionsFragment.class.getSimpleName();

    MapView mMapView;
    private View mProgressView;
    private GoogleMap googleMap;
    int mCurrentPosition = -1;

    private boolean initMap = true;
    private Intervention[] interventionTab;
    private Map<String, Marker> labelsMarkersHashMap = new HashMap<>();
    private LatLngBounds.Builder bounds;


    @Override
    public void refresh() {
        if (null != getActivity()){

            Log.i(TAG, "refresh, clearMapData");

            // clear lists
            clearMapData();
            // fill lists
            initMap();
        }
    }

    private void clearMapData(){
        for (Intervention intervention : interventionTab){
            if (labelsMarkersHashMap.get(intervention.getName()) != null){
                labelsMarkersHashMap.get(intervention.getName()).remove();
            }
        }
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {

        // inflat and return the layout
        View v = inflater.inflate(R.layout.map_view, container,
                false);

        mProgressView = v.findViewById(R.id.map_progress);
        mProgressView.setVisibility(View.VISIBLE);

        mMapView = (MapView) v.findViewById(R.id.mapView);
        mMapView.onCreate(savedInstanceState);
        mMapView.onResume();// needed to get the map to refresh immediately

        try {
            MapsInitializer.initialize(getActivity().getApplicationContext());
        } catch (Exception e) {
            e.printStackTrace();
        }

        googleMap = mMapView.getMap();

        initMap();

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
        if (position < interventionTab.length) {
            Intervention intervention = interventionTab[position];
            CameraPosition cameraPosition = new CameraPosition.Builder()
                    .target(new LatLng(intervention.getLatitude(), intervention.getLongitude())).zoom(16).build();
            googleMap.animateCamera(CameraUpdateFactory.newCameraPosition(cameraPosition));
            mCurrentPosition = position;
        } else {
            Log.e(TAG, "Error: position is out of list of interventions");
        }
    }

    public void initMap(){

        interventionTab = ((ListInterventionsActivity) getActivity()).getInterventions();
        // Create LatLngBound to zoom on the set of positions in the path
        bounds = new LatLngBounds.Builder();


        if(initMap) {
            initMap = false;
            mProgressView.setVisibility(View.INVISIBLE);
        }

        if (interventionTab.length > 0){
            for (Intervention intervention : interventionTab) {
                double latitude = intervention.getLatitude();
                double longitude = intervention .getLongitude();
                LatLng latLng = new LatLng(latitude, longitude);
                bounds.include(latLng);

                // create marker
                MarkerOptions marker = new MarkerOptions().position(
                        new LatLng(intervention.getLatitude(), intervention.getLongitude())).title("Hello Maps");
                // Changing marker icon
                marker.icon(BitmapDescriptorFactory
                        .defaultMarker(BitmapDescriptorFactory.HUE_ROSE));
                // adding marker
                Marker markerAdded = googleMap.addMarker(marker);

                labelsMarkersHashMap.put(intervention.getName(), markerAdded);
            }

        }
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
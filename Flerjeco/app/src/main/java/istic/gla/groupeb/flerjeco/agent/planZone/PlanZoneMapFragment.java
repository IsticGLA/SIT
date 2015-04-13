package istic.gla.groupeb.flerjeco.agent.planZone;

import android.os.AsyncTask;
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
import com.google.android.gms.maps.model.MarkerOptions;

import org.springframework.web.client.HttpStatusCodeException;

import java.util.ArrayList;
import java.util.List;

import entity.Path;
import entity.Position;
import entity.Resource;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.springRest.SpringService;
import util.ResourceCategory;

/**
 * A fragment that launches other parts of the demo application.
 */
public class PlanZoneMapFragment extends Fragment {

    final static String ARG_POSITION = "position";

    MapView mMapView;
    private GoogleMap googleMap;
    int mCurrentPosition = -1;
    boolean editMode = false;

    private List<Resource> droneList;
    private List<Path> pathList;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // inflate and return the layout
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

        CameraPosition cameraPosition = new CameraPosition.Builder()
                .target(new LatLng(48.11705, -1.63825)).zoom(16).build();

        googleMap.animateCamera(CameraUpdateFactory
                .newCameraPosition(cameraPosition));



        /*googleMap.setOnMapClickListener(new GoogleMap.OnMapClickListener() {
            @Override
            public void onMapClick(LatLng latLng) {
                if (editMode) {
                    double latitude = latLng.latitude;
                    double longitude = latLng.longitude;
                    Log.i(getActivity().getLocalClassName(), "Click on the Map at " + latitude + ", " + longitude);
                    new SendLocationToDrone().execute(latitude, longitude);
                }
            }
        });*/

        PlanZoneActivity activity = (PlanZoneActivity) getActivity();
        initMap(activity.getResourceEntities(), activity.getPaths());

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

    public void updateMapView(final int position) {
        googleMap.clear();
        if (null != pathList.get(position)){
            mCurrentPosition = position;
            Resource res = droneList.get(position);
            LatLngBounds.Builder bounds = new LatLngBounds.Builder();
            for (Position p : pathList.get(position).getPositions()){
                bounds.include(new LatLng(p.getLatitude(), p.getLongitude()));

                // create marker
                MarkerOptions marker = new MarkerOptions().position(
                        new LatLng(p.getLatitude(), p.getLongitude())).title(res.getLabel());
                // Changing marker icon
                marker.icon(BitmapDescriptorFactory
                        .defaultMarker(BitmapDescriptorFactory.HUE_AZURE));
                // adding marker
                googleMap.addMarker(marker);
            }
            googleMap.animateCamera(CameraUpdateFactory.newLatLngBounds(bounds.build(), 50));
        } else {
            Log.i("TEST", "test");
            googleMap.setOnMapClickListener(new GoogleMap.OnMapClickListener() {
                @Override
                public void onMapClick(LatLng latLng) {
                    double latitude = latLng.latitude;
                    double longitude = latLng.longitude;
                    Log.i("Click on the map", "latitude : " + latitude + ", " + "longitude : " + longitude);

                    // create marker
                    MarkerOptions marker = new MarkerOptions().position(
                            new LatLng(latitude, longitude)).title("new Path");
                    // Changing marker icon
                    marker.icon(BitmapDescriptorFactory
                            .defaultMarker(BitmapDescriptorFactory.HUE_AZURE));
                    // adding marker
                    googleMap.addMarker(marker);
                }
            });
        }
    }

    public void initMap(List<Resource> droneList, List<Path> pathList){
        this.droneList = new ArrayList<>();
        for (Resource res : droneList){
            if (res.getResourceCategory() == ResourceCategory.drone){
                this.droneList.add(res);

                // create marker
                MarkerOptions marker = new MarkerOptions().position(
                        new LatLng(res.getLatitude(), res.getLongitude())).title(res.getLabel());
                // Changing marker icon
                marker.icon(BitmapDescriptorFactory
                        .defaultMarker(BitmapDescriptorFactory.HUE_ROSE));
                // adding marker
                googleMap.addMarker(marker);
            }
        }
        this.pathList = pathList;
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

    /**
     * Represents an asynchronous call for move drone to location you clicked
     * the user.
     */
    public class SendLocationToDrone extends AsyncTask<Object, Void, Long> {

        @Override
        protected Long doInBackground(Object... params) {
            try {
                Long id = new SpringService().moveDrone(params);
                return id;

            } catch (HttpStatusCodeException e) {
                Log.e("Drone don't move", e.getMessage(), e);
            }

            return null;
        }

        @Override
        protected void onPostExecute(Long id) {
            Log.i("SendLocationToDrone", "Request posted");
        }
    }
}
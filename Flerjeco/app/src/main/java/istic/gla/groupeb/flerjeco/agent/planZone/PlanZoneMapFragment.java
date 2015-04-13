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
import com.google.android.gms.maps.model.CameraPosition;
import com.google.android.gms.maps.model.LatLng;

import org.springframework.web.client.HttpStatusCodeException;

import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.springRest.SpringService;

/**
 * A fragment that launches other parts of the demo application.
 */
public class PlanZoneMapFragment extends Fragment {

    MapView mMapView;
    private GoogleMap googleMap;

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



        googleMap.setOnMapClickListener(new GoogleMap.OnMapClickListener() {
            @Override
            public void onMapClick(LatLng latLng) {
            double latitude = latLng.latitude;
            double longitude = latLng.longitude;
            Log.i(getActivity().getLocalClassName(), "Click on the Map at "+latitude+", "+longitude);
            new SendLocationToDrone().execute(latitude, longitude);

            }
        });

        return v;
    }

    @Override
    public void onStart() {
        super.onStart();
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
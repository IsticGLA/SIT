package istic.gla.groupeb.flerjeco.agent.intervention;

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
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import entity.Intervention;
import entity.Resource;
import istic.gla.groupeb.flerjeco.R;
import util.State;

/**
 * A fragment that launches other parts of the demo application.
 */
public class InterventionMapFragment extends Fragment {

    final static String ARG_POSITION = "position";

    MapView mMapView;
    private GoogleMap googleMap;

    int position = -1;

    private Intervention intervention;
    private List<Resource> resources = new ArrayList<>();
    private List<Resource> resourcesToPutOnMap = new ArrayList<>();
    private Map<String, Marker> markers = new HashMap<>();


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

        InterventionActivity interventionActivity = (InterventionActivity) getActivity();
        initMap(interventionActivity.intervention);

        googleMap.setOnMapClickListener(new GoogleMap.OnMapClickListener() {
            @Override
            public void onMapClick(LatLng latLng) {
                double latitude = latLng.latitude;
                double longitude = latLng.longitude;
                Log.i(getActivity().getLocalClassName(), "Click on the Map at " + latitude + ", " + longitude + " for item "+position);
                // create marker
                MarkerOptions marker = new MarkerOptions().position(
                        new LatLng(latitude, longitude)).title(resourcesToPutOnMap.get(position).getLabel());
                // Changing marker icon
                marker.icon(BitmapDescriptorFactory
                        .defaultMarker(BitmapDescriptorFactory.HUE_AZURE));

                if (markers.get(resourcesToPutOnMap.get(position).getLabel())!=null){
                    markers.get(resourcesToPutOnMap.get(position).getLabel()).remove();
                }
                // adding marker
                Marker markerAdded = googleMap.addMarker(marker);
                markers.put(resourcesToPutOnMap.get(position).getLabel(),markerAdded);

            }
        });

        return v;
    }

    public void initMap(Intervention intervention){
        this.intervention = intervention;

        if (intervention.getResources().size()>0){

            for (Resource resource : intervention.getResources()){
                State resourceState = resource.getState();
                if (State.active.equals(resourceState) || State.planned.equals(resourceState)){
                    // create marker
                    MarkerOptions marker = new MarkerOptions().position(
                            new LatLng(resource.getLatitude(), resource.getLongitude())).title(resource.getLabel());
                    // Changing marker icon
                    marker.icon(BitmapDescriptorFactory
                            .defaultMarker(BitmapDescriptorFactory.HUE_ROSE));
                    // adding marker
                    googleMap.addMarker(marker);

                    resources.add(resource);
                }else if (State.validated.equals(resourceState)){
                    resourcesToPutOnMap.add(resource);
                }

            }

        }

        CameraPosition cameraPosition = new CameraPosition.Builder()
                .target(new LatLng(intervention.getLatitude(), intervention.getLongitude())).zoom(12).build();

        googleMap.animateCamera(CameraUpdateFactory
                .newCameraPosition(cameraPosition));
    }

    public int getPosition() {
        return position;
    }

    public void setPosition(int position) {
        this.position = position;
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
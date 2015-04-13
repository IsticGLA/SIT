package istic.gla.groupeb.flerjeco.agent.intervention;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
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

import java.util.ArrayList;
import java.util.List;

import entity.Intervention;
import entity.Marker;
import entity.Resource;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.icons.Vehicle;
import util.ResourceRole;
import util.State;

/**
 * A fragment that launches other parts of the demo application.
 */
public class MapFragment extends Fragment {

    final static String ARG_POSITION = "position";

    MapView mMapView;
    private GoogleMap googleMap;
    int mCurrentPosition = -1;

    private Intervention intervention;
    private List<Resource> resources = new ArrayList<>();

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

        SecondActivity secondActivity = (SecondActivity) getActivity();
        initMap(secondActivity.intervention);

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
        Resource resource = resources.get(position);
        if (resource.getState() == State.planned || resource.getState() == State.active) {
            CameraPosition cameraPosition = new CameraPosition.Builder()
                    .target(new LatLng(resource.getLatitude(), resource.getLongitude())).zoom(16).build();
            googleMap.animateCamera(CameraUpdateFactory
                    .newCameraPosition(cameraPosition));
            mCurrentPosition = position;
        }
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
                    //marker.icon(BitmapDescriptorFactory
                    //        .defaultMarker(BitmapDescriptorFactory.HUE_ROSE));

                    drawMarker(marker, resource);
                    // adding marker
                    googleMap.addMarker(marker);

                    resources.add(resource);
                }

            }
        }

        CameraPosition cameraPosition = new CameraPosition.Builder()
                .target(new LatLng(intervention.getLatitude(), intervention.getLongitude())).zoom(12).build();

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

    public void drawMarker(MarkerOptions markerOptions, Resource resource){
        //switch (resource.)
        Vehicle mVehicle = new Vehicle(resource.getLabel(), ResourceRole.people, resource.getState());
        int width = mVehicle.getRect().width();
        int height = mVehicle.getRect().height()+mVehicle.getRect2().height()+10;
        Bitmap mBitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        Canvas mCanvas = new Canvas(mBitmap);
        mVehicle.drawVehicle(mCanvas);
        markerOptions.icon(BitmapDescriptorFactory.fromBitmap(mBitmap));
    }
}
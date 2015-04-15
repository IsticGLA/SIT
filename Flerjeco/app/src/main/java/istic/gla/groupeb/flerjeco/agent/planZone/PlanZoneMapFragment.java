package istic.gla.groupeb.flerjeco.agent.planZone;

import android.graphics.Color;
import android.os.AsyncTask;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Toast;

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
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;

import java.util.ArrayList;
import java.util.List;

import entity.Intervention;
import entity.Path;
import entity.Position;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.springRest.SpringService;

/**
 * A fragment that launches other parts of the demo application.
 */
public class PlanZoneMapFragment extends Fragment {

    private static final String TAG = PlanZoneMapFragment.class.getSimpleName();
    final static String ARG_POSITION = "position";

    MapView mMapView;
    private GoogleMap googleMap;
    private List<Polyline> polylines;
    private List<Marker> markers;
    int mCurrentPosition = -1;
    private List<Path> pathList;

    public Path newPath;
    private Path savePath;

    public boolean removePath = false;
    public boolean editPath = false;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // inflate and return the layout
        View v = inflater.inflate(R.layout.map_view, container,
                false);
        mMapView = (MapView) v.findViewById(R.id.mapView);
        mMapView.onCreate(savedInstanceState);

        mMapView.onResume();// needed to get the map to refresh immediately

        try {
            MapsInitializer.initialize(getActivity().getApplicationContext());
        } catch (Exception e) {
            e.printStackTrace();
        }

        googleMap = mMapView.getMap();

        PlanZoneActivity activity = (PlanZoneActivity) getActivity();

        // Center the camera on the intervention position
        CameraPosition cameraPosition = new CameraPosition.Builder()
                .target(new LatLng(activity.getIntervention().getLatitude(), activity.getIntervention().getLongitude())).zoom(16).build();
        googleMap.animateCamera(CameraUpdateFactory
                .newCameraPosition(cameraPosition));

        initMap(activity.getPaths());

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

    /**
     * Creation of the new path
     */
    public void createPath(){
        // clear the Google Map
        clearGoogleMap();
        editPath = false;
        addMapClickListener();
    }

    /**
     * Update the Google Map with the path you just clicked
     * @param position position of the path you clicked in the list
     */
    public void updateMapView(int position) {
        // clear the Google Map
        clearGoogleMap();

        // if path of the position position in the list is not null, we draw it on the map
        if (position < pathList.size() && null != pathList.get(position)){
            // Set mCurrentPosition to future resume on fragment
            mCurrentPosition = position;


            // set closed property on newPath
            boolean b = pathList.get(position).isClosed();
            newPath.setClosed(b);
            ((PlanZoneActivity) getActivity()).checkCloseBox(b);

            List<Position> positions = pathList.get(position).getPositions();

            // save the old path for future restore if the update doesn't work
            savePath = new Path();
            savePath.setClosed(b);

            // Create LatLngBound to zoom on the set of positions in the path
            LatLngBounds.Builder bounds = new LatLngBounds.Builder();
            for (int i = 0; i < positions.size(); i++){
                double latitude = positions.get(i).getLatitude();
                double longitude = positions.get(i).getLongitude();
                LatLng latLng = new LatLng(latitude, longitude);
                bounds.include(latLng);

                newPath.getPositions().add(new Position(latitude, longitude, 20));
                savePath.getPositions().add(new Position(latitude, longitude, 20));

                // create marker
                MarkerOptions marker = new MarkerOptions().position(latLng);
                // Changing marker icon
                marker.icon(BitmapDescriptorFactory
                        .defaultMarker(BitmapDescriptorFactory.HUE_AZURE));
                // adding marker
                Marker m = googleMap.addMarker(marker);

                // add the marker on the markers list
                markers.add(m);

                // draw line between two points if is not the first
                if (i > 0){
                    LatLng previousLatLng = new LatLng(positions.get(i-1).getLatitude(), positions.get(i-1).getLongitude());
                    drawLine(latLng, previousLatLng);
                // else if it the path is closed, draw line between first and last point
                } if (i == positions.size()-1 && pathList.get(position).isClosed()){
                    LatLng firstLatLng = new LatLng(positions.get(0).getLatitude(), positions.get(0).getLongitude());
                    drawLine(firstLatLng, latLng);
                }
            }
            googleMap.animateCamera(CameraUpdateFactory.newLatLngBounds(bounds.build(), 50));
        }
    }

    /**
     * Init the Google Map
     * @param pathList
     */
    public void initMap(List<Path> pathList){
        this.pathList = pathList;
        this.polylines = new ArrayList<>();
        this.markers = new ArrayList<>();
        this.newPath = new Path();
    }

    public void clearGoogleMap(){
        googleMap.clear();
        this.polylines = new ArrayList<>();
        this.markers = new ArrayList<>();
        this.newPath = new Path();
    }

    /**
     * Send the new path in the database
     */
    public void sendPath(){
        resetMapListener();
        Intervention inter = ((PlanZoneActivity)getActivity()).getIntervention();
        if (editPath){
            inter.getWatchPath().get(mCurrentPosition).setPositions(newPath.getPositions());
            inter.getWatchPath().get(mCurrentPosition).setClosed(newPath.isClosed());
        } else {
            inter.getWatchPath().add(newPath);
        }
        new SendPathToDrone().execute(inter);
    }

    /**
     * Remove path on the database
     */
    public void removePath(){
        removePath = true;
        resetMapListener();
        Intervention inter = ((PlanZoneActivity)getActivity()).getIntervention();
        inter.getWatchPath().remove(mCurrentPosition);
        new SendPathToDrone().execute(inter);
    }

    /**
     * Add Map Click listener on the Google Map
     */
    public void addMapClickListener(){
        // Add clickListener on the map two save positions in new path
        googleMap.setOnMapClickListener(new GoogleMap.OnMapClickListener() {
            @Override
            public void onMapClick(LatLng latLng) {
                double latitude = latLng.latitude;
                double longitude = latLng.longitude;
                Log.i("Click on the map", "latitude : " + latitude + ", " + "longitude : " + longitude);

                //if there are at least two polyline on the Google Map and newPath is closed, remove the last one
                if (polylines.size() > 2 && newPath.isClosed()) {
                    removeLine(polylines.size() - 1);
                }

                // create marker
                MarkerOptions marker = new MarkerOptions().position(
                        new LatLng(latitude, longitude)).title("new Path");
                // Changing marker icon
                marker.icon(BitmapDescriptorFactory
                        .defaultMarker(BitmapDescriptorFactory.HUE_AZURE));
                // adding marker
                Marker m = googleMap.addMarker(marker);

                // add the marker on the markers list
                markers.add(m);

                // Set the Position on newPath
                newPath.getPositions().add(new Position(latitude, longitude, 20));

                int size = newPath.getPositions().size();
                // draw line between two points if is not the first
                if (size > 1) {
                    LatLng previousLatLng = new LatLng(newPath.getPositions().get(size - 2).getLatitude(), newPath.getPositions().get(size - 2).getLongitude());
                    drawLine(latLng, previousLatLng);
                    // else if it the path is closed, draw line between first and last point
                }
                if (size > 2 && newPath.isClosed()) {
                    LatLng firstLatLng = new LatLng(newPath.getPositions().get(0).getLatitude(), newPath.getPositions().get(0).getLongitude());
                    drawLine(firstLatLng, latLng);
                }
            }
        });
    }

    /**
     * Reset the MapClickListener on the Google Map
     */
    public void resetMapListener(){
        googleMap.setOnMapClickListener(null);
    }

    /**
     * function called when we check or uncheck the checkbox checkbox_closed_path
     */
    public void closePath(){
        if (!newPath.isClosed()) {
            Log.i(TAG, "Close the new path");
            newPath.setClosed(true);

            drawClosePolyline();
        } else {
            Log.i(TAG, "Open the new path");
            newPath.setClosed(false);

            // if there are at least three points in newPath
            if (newPath.getPositions().size() > 2) {
                removeLine(polylines.size() - 1);
            }
        }
    }

    /**
     * Draw the polyline which close the path
     */
    public void drawClosePolyline(){
        // if there are at least two points in the path
        if (newPath.getPositions().size() > 2) {
            LatLng firstLatLng = new LatLng(newPath.getPositions().get(0).getLatitude(), newPath.getPositions().get(0).getLongitude());
            LatLng lastLatLng = new LatLng(newPath.getPositions().get(newPath.getPositions().size() - 1).getLatitude(),
                    newPath.getPositions().get(newPath.getPositions().size() - 1).getLongitude());
            drawLine(firstLatLng, lastLatLng);
        }
    }

    /**
     * Drw polyline on the Google Map from first and last LatLng
     * @param first start point of the polyline to draw
     * @param last last point of the polyline to draw
     */
    public void drawLine(LatLng first, LatLng last){
        Polyline line = googleMap.addPolyline((new PolylineOptions())
                .add(first, last).width(3).color(Color.BLUE)
                .geodesic(true));
        polylines.add(line);
    }

    /**
     * Remove the polyline on the Google Map
     * @param i indice of the polyline to remove in the polylines list
     */
    public void removeLine(int i){
        Log.i(TAG, "Remove the polyline at the " + i + " position");
        polylines.get(i).remove();
        polylines.remove(i);
    }

    /**
     * Remove the last point of the new path
     */
    public void removeLastPoint(){
        Log.i(TAG, "Remove the last position on the path");
        // remove the last marker on the Google Map
        int i = markers.size()-1;
        markers.get(i).remove();
        markers.remove(i);
        newPath.getPositions().remove(i);

        // remove the last polyline if there is at least one polyline
        if (polylines.size() > 0) {
            removeLine(polylines.size() - 1);
        }
        // remove the polyline which close the path is isClosed is true
        if (polylines.size() > 1 && newPath.isClosed()){
            removeLine(polylines.size() - 1);
            // closed the path
            drawClosePolyline();
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

    /**
     * Represents an asynchronous call for add new path for drone in the current intervention
     * the user.
     */
    public class SendPathToDrone extends AsyncTask<Intervention, Void, Intervention> {

        private final String TAG = SendPathToDrone.class.getSimpleName();

        @Override
        protected Intervention doInBackground(Intervention... params) {
            try {
                Log.i(TAG, "Update of the intervention with id : "+ params[0].getId());
                SpringService springService = new SpringService();
                Intervention intervention = springService.updateIntervention(params[0]);
                return intervention;

            } catch (Exception e) {
                Log.e(TAG, e.getMessage(), e);
            }

            return null;
        }

        @Override
        protected void onPostExecute(Intervention intervention) {
            if (null == intervention){
                Log.i(TAG, "La mise à jour de l'intervention n'a pas fonctionnée, veuillez rééssayer");
                Intervention i = ((PlanZoneActivity) getActivity()).getIntervention();

                // update doesn't work
                if (editPath && !removePath){
                    Log.i("TEST", "Update");
                    pathList.set(mCurrentPosition, savePath);
                    updateMapView(mCurrentPosition);

                    // remove doesn't work
                } else if (removePath) {
                    Log.i("TEST", "Delete " + savePath.toString() + "  " + mCurrentPosition);
                    pathList.add(mCurrentPosition, savePath);
                    updateMapView(mCurrentPosition);

                    // create doesn't work
                } else {
                    Log.i("TEST", "Create");
                    pathList.remove(pathList.size()-1);
                    clearGoogleMap();
                }
                Toast.makeText(getActivity().getApplicationContext(), "La mise à jour de l'intervention n'a pas fonctionnée, veuillez rééssayer", Toast.LENGTH_LONG).show();
            } else {
                Log.i(TAG, "Intervention was updated !");
                Toast.makeText(getActivity().getApplicationContext(), "Les modifications ont été enregistées", Toast.LENGTH_LONG).show();
                pathList = intervention.getWatchPath();
                updateMapView(intervention.getWatchPath().size()-1);
                ((PlanZoneActivity) getActivity()).refreshList(intervention);
            }
        }
    }
}
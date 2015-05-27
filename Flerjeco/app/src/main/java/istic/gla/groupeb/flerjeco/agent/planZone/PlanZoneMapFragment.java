package istic.gla.groupeb.flerjeco.agent.planZone;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.util.Pair;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.MapView;
import com.google.android.gms.maps.MapsInitializer;
import com.google.android.gms.maps.model.BitmapDescriptor;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.CameraPosition;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.LatLngBounds;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.Polygon;
import com.google.android.gms.maps.model.PolygonOptions;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;

import org.apache.commons.collections4.CollectionUtils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import istic.gla.groupb.nivimoju.entity.Area;
import istic.gla.groupb.nivimoju.entity.Drone;
import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Path;
import istic.gla.groupb.nivimoju.entity.Position;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.agent.DronesMapFragment;
import istic.gla.groupeb.flerjeco.springRest.GetPositionDroneTask;
import istic.gla.groupeb.flerjeco.springRest.UpdateAreaTask;
import istic.gla.groupeb.flerjeco.synch.IntentWraper;
import istic.gla.groupeb.flerjeco.utils.LatLngUtils;

/**
 * Fragment de carte pour les drones
 */
public class PlanZoneMapFragment extends Fragment implements DronesMapFragment {

    private static final String TAG = PlanZoneMapFragment.class.getSimpleName();
    final static String ARG_POSITION = "position";

    ECreationType type = ECreationType.PATH;

    // all variables for the Google Map
    MapView mMapView;
    private GoogleMap googleMap;
    private List<Pair<Polyline, Marker>> polylines;
    private List<Marker> markers;

    private PolygonOptions polygonOptions;
    private List<LatLng> polygonPoints;
    private Polygon polygon;

    // list of all the path of the intervention
    private List<Path> pathList;
    // current position in the pathList in the intervention
    int mCurrentPosition = -1;
    // initialized when we create a new path for the current intervention
    public Path newPath;
    // save the path when we edit it to future restore (request to database failed)
    private Path savePath;

    // list of all the area of the intervention
    private List<Area> areaList;
    // current position in the areaList in the intervention
    int mCurrentPositionArea = -1;
    // initialized when we create a new area for the current intervention
    public Area newArea;

    // current intervention
    private Intervention inter;

    // indicate if we want to remove a path in the intervention
    public boolean removePath = false;
    // indicate if we want to edit a path in the intervention
    public boolean editPath = false;

    private Map<String, Marker> dronesMarkers;
    private boolean refreshDrones = false;
    GetPositionDroneTask getPositionDroneTask;
    UpdatePathsForInterventionTask updatePathsForInter;
    UpdateAreaTask updateAreaTask;


    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // inflate and return the layout
        View v = inflater.inflate(R.layout.map_view, container, false);
        mMapView = (MapView) v.findViewById(R.id.mapView);
        mMapView.onCreate(savedInstanceState);

        // needed to get the map to refresh immediately
        mMapView.onResume();

        // Initialize Google Map
        try {
            MapsInitializer.initialize(getActivity().getApplicationContext());
        } catch (Exception e) {
            e.printStackTrace();
        }

        // get the googleMap
        googleMap = mMapView.getMap();

        PlanZoneActivity activity = (PlanZoneActivity) getActivity();

        // Center the camera on the intervention position
        CameraPosition cameraPosition = new CameraPosition.Builder()
                .target(new LatLng(activity.getIntervention().getLatitude(), activity.getIntervention().getLongitude())).zoom(16).build();
        googleMap.animateCamera(CameraUpdateFactory
                .newCameraPosition(cameraPosition));

        initMap(activity.getIntervention().getWatchPath(), activity.getIntervention().getWatchArea());

        this.dronesMarkers = new HashMap<>();
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
            updateMapView(args.getInt(ARG_POSITION), ECreationType.PATH);
        } else if (mCurrentPosition != -1) {
            // Set article based on saved instance state defined during onCreateView
            updateMapView(mCurrentPosition, ECreationType.PATH);
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
    public void updateMapView(int position, ECreationType type) {
        // clear the Google Map
        clearGoogleMap();
        this.type = type;

        // get the intervention from the main activity
        inter = ((PlanZoneActivity) getActivity()).getIntervention();
        Log.i("JVG", type.toString());
        if (type == ECreationType.PATH) {

            // if path of the position position in the list is not null, we draw it on the map
            if (pathList.size() > 0 && position < pathList.size() && null != pathList.get(position)) {
                // Set mCurrentPosition to future resume on fragment
                mCurrentPosition = position;


                // set closed property on newPath
                boolean b = pathList.get(position).isClosed();
                newPath.setClosed(b);
                ((PlanZoneActivity) getActivity()).checkCloseBox(b);

                // save the old path for future restore if the update doesn't work
                savePath = new Path();
                savePath.setClosed(b);

                // get all the positions of the path to draw it
                List<Position> positions = pathList.get(position).getPositions();

                // Create LatLngBound to zoom on the set of positions in the path
                LatLngBounds.Builder bounds = new LatLngBounds.Builder();
                for (int i = 0; i < positions.size(); i++) {
                    double latitude = positions.get(i).getLatitude();
                    double longitude = positions.get(i).getLongitude();
                    LatLng latLng = new LatLng(latitude, longitude);
                    bounds.include(latLng);

                    newPath.getPositions().add(new Position(latitude, longitude, 20));
                    savePath.getPositions().add(new Position(latitude, longitude, 20));

                    // create marker
                    MarkerOptions marker = new MarkerOptions().position(latLng);
                    // Changing marker icon
                    marker.icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_VIOLET));
                    // adding marker
                    Marker m = googleMap.addMarker(marker);
                    // add the marker on the markers list
                    markers.add(m);

                    // draw line between two points if is not the first
                    if (i > 0) {
                        LatLng previousLatLng = new LatLng(positions.get(i - 1).getLatitude(), positions.get(i - 1).getLongitude());
                        drawLine(latLng, previousLatLng);
                        // else if it the path is closed, draw line between first and last point
                    }
                    if (i == positions.size() - 1 && pathList.get(position).isClosed() && positions.size() > 2) {
                        LatLng firstLatLng = new LatLng(positions.get(0).getLatitude(), positions.get(0).getLongitude());
                        drawLine(firstLatLng, latLng);
                    }
                }
                googleMap.animateCamera(CameraUpdateFactory.newLatLngBounds(bounds.build(), 50));
            }
        } else {
            if (areaList.size() > 0 && position < areaList.size() && null != areaList.get(position)){
                mCurrentPositionArea = position;

                newArea = areaList.get(position);
                LatLngBounds.Builder bounds = new LatLngBounds.Builder();

                for (Position pos : newArea.getPositions()) {
                    LatLng latLng = new LatLng(pos.getLatitude(), pos.getLongitude());
                    bounds.include(latLng);

                    // create marker
                    MarkerOptions marker = new MarkerOptions().position(latLng);
                    // Changing marker icon
                    marker.icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_VIOLET));
                    // adding marker
                    Marker m = googleMap.addMarker(marker);
                    // add the marker on the markers list
                    markers.add(m);

                    polygonOptions.add(latLng);
                    polygonPoints.add(latLng);
                }
                polygon = googleMap.addPolygon(polygonOptions);

                googleMap.animateCamera(CameraUpdateFactory.newLatLngBounds(bounds.build(), 50));
            }
        }
    }

    /**
     * Init the Google Map
     * @param pathList the list of paths
     */
    public void initMap(List<Path> pathList, List<Area> areaList){
        this.pathList = pathList;
        this.areaList = areaList;
        this.polylines = new ArrayList<>();
        this.markers = new ArrayList<>();
        this.dronesMarkers = new HashMap<>();
        this.newPath = new Path();
        this.newArea = new Area();
        this.polygonOptions = new PolygonOptions()
                .strokeColor(Color.parseColor("#9b24a6"))
                .fillColor(Color.argb(40,238,238,0))
                .strokeWidth((float) 2);
        this.polygonPoints = new ArrayList<>();
    }

    /**
     * Clear the Google Map
     */
    public void clearGoogleMap(){
        googleMap.clear();
        this.polylines.clear();
        this.markers.clear();
        this.newPath = new Path();
        this.newArea = new Area();
        this.dronesMarkers.clear();
        if (this.polygon!=null){
            this.polygon.remove();
            this.polygon = null;
        }
        this.polygonOptions = new PolygonOptions()
                .strokeColor(Color.parseColor("#9b24a6"))
                .fillColor(Color.argb(40,238,238,0))
                .strokeWidth((float) 2);;
        this.polygonPoints.clear();
    }

    /**
     * Send the new path to the server
     */
    public void sendPath(){
        ((PlanZoneActivity)getActivity()).showProgress(true);
        // remove Click listener
        resetMapListener();

        // if we are in edition mode, we set the new path in the intervention we get back from the main activity
        if (editPath){

            // send to the database
            Path pathToUpdate = inter.getWatchPath().get(mCurrentPosition);
            pathToUpdate.setPositions(newPath.getPositions());
            pathToUpdate.setClosed(newPath.isClosed());
            Object[] tab = {inter.getId(), pathToUpdate};
            updatePathsForInter = new UpdatePathsForInterventionTask(this, EPathOperation.UPDATE);
            updatePathsForInter.execute(tab);

        // else, we add the new path
        } else {
            Object[] tab = {inter.getId(), newPath};
            updatePathsForInter = new UpdatePathsForInterventionTask(this, EPathOperation.CREATE);
            updatePathsForInter.execute(tab);
        }
    }

    /**
     * Remove path on the database
     * @param creationType
     */
    public void removePath(ECreationType creationType){
        removePath = true;
        // remove Click listener
        resetMapListener();
        // get back the intervention from the main activity
        inter = ((PlanZoneActivity)getActivity()).getIntervention();

        if(creationType == ECreationType.PATH) {
            // remove of the path we want to remove
            if (mCurrentPosition >= 0 && inter.getWatchPath().size() > mCurrentPosition) {
                //inter.getWatchPath().remove(mCurrentPosition);
                // send to the database
                Object[] tab = {inter.getId(), inter.getWatchPath().get(mCurrentPosition)};
                updatePathsForInter = new UpdatePathsForInterventionTask(this, EPathOperation.DELETE);
                updatePathsForInter.execute(tab);
            }
        } else if (creationType == ECreationType.AREA) {
            // remove of the area we want to remove
            if (mCurrentPosition >= 0 && inter.getWatchArea().size() > mCurrentPosition) {
                //inter.getWatchPath().remove(mCurrentPosition);
                // send to the database
                Object[] tab = {inter.getId(), inter.getWatchPath().get(mCurrentPosition)};
                updateAreaTask = new UpdateAreaTask(this, EPathOperation.DELETE);
                updateAreaTask.execute(tab);
            }
        }
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

                //if there are at least two polyline on the Google Map and newPath is closed, remove the last one
                if (polylines.size() > 2 && newPath.isClosed()) {
                    removeLine(polylines.size() - 1);
                }

                // create marker
                MarkerOptions marker = new MarkerOptions().position(
                        new LatLng(latitude, longitude)).title("new Path");
                // Changing marker icon
                marker.icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_VIOLET));
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
                }
                // else if it the path is closed, draw line between first and last point
                if (size > 2 && newPath.isClosed()) {
                    LatLng firstLatLng = new LatLng(newPath.getPositions().get(0).getLatitude(), newPath.getPositions().get(0).getLongitude());
                    drawLine(firstLatLng, latLng);
                }
                ((PlanZoneActivity) getActivity()).enableCreatePathButton();
            }
        });
    }

    /**
     * callback for when the path create/delete/update succeed
     */
    public void applyUpdateAfterOperation(Intervention intervention){
        pathList = intervention.getWatchPath();
        areaList = intervention.getWatchArea();
        if (this.type == ECreationType.PATH) {
            updateMapView(intervention.getWatchPath().size() - 1, this.type);
        } else {
            updateMapView(intervention.getWatchArea().size() - 1, this.type);
        }
        PlanZoneActivity p = ((PlanZoneActivity) getActivity());
        p.refreshList(intervention);
        p.editModeOff();
        ((PlanZoneActivity)getActivity()).showProgress(false);
    }

    /**
     * callback for when the path create/delete/update succeed
     */
    public void refreshMapAfterSynchro(Intervention newIntervention, Intervention oldIntervention){
        PlanZoneActivity p = ((PlanZoneActivity) getActivity());
        pathList = newIntervention.getWatchPath();
        if (p.isEditionMode()){
            if (mCurrentPosition != -1 && pathList.size() > 0) {
                Path oldPath = findPathById(oldIntervention.getWatchPath().get(mCurrentPosition).getIdPath(), oldIntervention.getWatchPath());
                Path updatePath = findPathById(oldIntervention.getWatchPath().get(mCurrentPosition).getIdPath(), newIntervention.getWatchPath());
                // remove a path
                if (updatePath == null) {
                    if (newIntervention.getWatchPath().size() > 0) {
                        updateMapView(newIntervention.getWatchPath().size() - 1, ECreationType.PATH);
                        p.checkListView(newIntervention.getWatchPath().size() - 1);
                    }
                // update path
                } else if (oldPath.isClosed() != updatePath.isClosed() || !oldPath.getPositions().equals(updatePath.getPositions())){
                    updateMapView(mCurrentPosition, ECreationType.PATH);
                    p.checkListView(mCurrentPosition);
                } else {
                    p.checkListView(mCurrentPosition);
                }
            }
        }
    }

    public Path findPathById(long idPath, List<Path> list){
        for (Path p : list){
            if (p.getIdPath() == idPath){
                return p;
            }
        }
        return null;
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
        // if the path is not close, we close it
        if (!newPath.isClosed()) {
            Log.i(TAG, "Close the path");
            newPath.setClosed(true);
            drawClosePolyline();
        // else unclose it
        } else {
            Log.i(TAG, "Open the path");
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
        // First you need rotate the bitmap of the arrowhead somewhere in your code
        float rotationDegrees = (float) LatLngUtils.angleFromCoordinate(last.latitude, last.longitude, first.latitude, first.longitude);

        // Create the rotated arrowhead bitmap
        Bitmap arrow = BitmapFactory.decodeResource(getResources(), R.drawable.arrow);
        BitmapDescriptor bitmapDescriptorFactory = BitmapDescriptorFactory.fromBitmap(arrow);
        // Get the middle position
        LatLng middlePos = LatLngUtils.midPoint(first.latitude, first.longitude, last.latitude, last.longitude);
        // Now we are gonna to add a marker
        Marker mArrowhead = googleMap.addMarker(new MarkerOptions()
                .position(middlePos)
                .flat(true)
                .anchor(0.5f, 0.5f)
                .rotation(rotationDegrees)
                .icon(bitmapDescriptorFactory));

        Polyline line = googleMap.addPolyline((new PolylineOptions())
                .add(first, last).width(3).color(Color.parseColor("#9b24a6"))
                .geodesic(true));

        polylines.add(new Pair<Polyline, Marker>(line, mArrowhead));
    }

    /**
     * Remove the polyline on the Google Map
     * @param i indice of the polyline to remove in the polylines list
     */
    public void removeLine(int i){
        Log.i(TAG, "Remove the polyline at the " + i + " position");
        if (polylines.size() > 0) {
            polylines.get(i).first.remove();
            polylines.get(i).second.remove();
            polylines.remove(i);
        }
    }

    /**
     * Remove the last point of the new path
     * @param creationType
     */
    public void removeLastPoint(ECreationType creationType){
        Log.i(TAG, "Remove the last position on the path");
        if (markers.size() > 0) {

            if(creationType == ECreationType.PATH) {
                // remove the last marker on the Google Map
                int i = markers.size() - 1;
                markers.get(i).remove();
                markers.remove(i);
                newPath.getPositions().remove(i);

                // remove the last polyline if there is at least one polyline
                if (polylines.size() > 0) {
                    removeLine(polylines.size() - 1);
                }
                // remove the polyline which close the path is isClosed is true
                if (polylines.size() > 1 && newPath.isClosed()) {
                    removeLine(polylines.size() - 1);
                    // closed the path
                    drawClosePolyline();
                }
            } else if (creationType == ECreationType.AREA) {
                int i = markers.size() - 1;
                markers.get(i).remove();
                markers.remove(i);
                newArea.getPositions().remove(i);
                polygonOptions = new PolygonOptions()
                        .strokeColor(Color.parseColor("#9b24a6"))
                        .fillColor(Color.argb(40,238,238,0))
                        .strokeWidth((float) 2);

                polygonPoints.remove(i);
                polygonOptions.addAll(polygonPoints);
                polygon.remove();
                polygon = null;
                if (i>0) {
                    polygon = googleMap.addPolygon(polygonOptions);
                }
            }

            if(markers.size() == 0) {
                ((PlanZoneActivity)getActivity()).disableCreatePathButton();
                ((PlanZoneActivity)getActivity()).disableCreateAreaButton();
            }
        }
    }

    // private variable for reducing object creations
    private Set<String> labels = new HashSet<>();
    /**
     * Show the marker for the drone of the intervention
     */
    @Override
    public void showDrones(Drone[] tab){
        if(refreshDrones) {
            labels.clear();
            for(Drone drone : tab){
                labels.add(drone.getLabel());
                if(dronesMarkers.containsKey(drone.getLabel())){
                    dronesMarkers.get(drone.getLabel()).setPosition(new LatLng(drone.getLatitude(), drone.getLongitude()));
                } else{
                    //create a new marker
                    Marker m = googleMap.addMarker(new MarkerOptions()
                            .position(new LatLng(drone.getLatitude(), drone.getLongitude()))
                            .title(drone.getLabel())
                            .anchor(0.5f, 0.5f)
                            .icon(BitmapDescriptorFactory.fromResource(R.drawable.ic_drone)));
                    dronesMarkers.put(drone.getLabel(), m);
                }
            }
            //delete marker for unassigned drones
            if(!labels.containsAll(dronesMarkers.keySet())){
                for(String labelToRemove : CollectionUtils.removeAll(dronesMarkers.keySet(), labels)){
                    dronesMarkers.get(labelToRemove).remove(); //remove the marker
                    dronesMarkers.remove(labelToRemove);
                }
            }
        }
    }

    @Override
    public void onResume() {
        super.onResume();
        inter = ((PlanZoneActivity)getActivity()).getIntervention();
        mMapView.onResume();

        long id = ((PlanZoneActivity)getActivity()).getIntervention().getId();
        refreshDrones = true;
        getPositionDroneTask = new GetPositionDroneTask(this, id);
        getPositionDroneTask.execute();
    }

    @Override
    public void onPause() {
        super.onPause();
        refreshDrones = false;
        if (null != getPositionDroneTask) {
            getPositionDroneTask.cancel(true);
        }
        mMapView.onPause();
        if (null != updatePathsForInter) {
            updatePathsForInter.cancel(true);
        }
        IntentWraper.stopService();
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

    @Override
    public void refreshDrones() {
        if(inter != null && refreshDrones) {
            getPositionDroneTask = new GetPositionDroneTask(this, inter.getId());
            getPositionDroneTask.execute();
        }else{
            Log.e(TAG, "inter : " + inter + "refreshDrones : " + refreshDrones);
        }
    }


    /**
     * Creation of the new path
     */
    public void createArea(){
        // clear the Google Map
        clearGoogleMap();
        editPath = false;
        addMapClickListenerArea();
    }

    /**
     * Add Map Click listener on the Google Map
     */
    public void addMapClickListenerArea(){
        // Add clickListener on the map two save positions in new path
        googleMap.setOnMapClickListener(new GoogleMap.OnMapClickListener() {
            @Override
            public void onMapClick(LatLng latLng) {

                // create marker
                MarkerOptions marker = new MarkerOptions().position(latLng);
                // Changing marker icon
                marker.icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_VIOLET));
                // adding marker
                Marker m = googleMap.addMarker(marker);
                // add the marker on the markers list
                markers.add(m);
                newArea.addPosition(new Position(latLng.latitude, latLng.longitude, 20));

                if (polygonOptions == null) {
                    polygonOptions = new PolygonOptions()
                            .strokeColor(Color.parseColor("#9b24a6"))
                            .fillColor(Color.argb(40, 238, 238, 0))
                            .strokeWidth((float) 2);
                }

                if (polygon != null) {
                    polygon.remove();
                    polygon = null;
                }

                polygonOptions.add(latLng);
                polygonPoints.add(latLng);

                polygon = googleMap.addPolygon(polygonOptions);
                ((PlanZoneActivity) getActivity()).enableCreateAreaButton();
            }
        });
    }

    public void sendArea() {

        // if we are in edition mode, we set the new path in the intervention we get back from the main activity
        if (editPath){

            // send to the database
            Path pathToUpdate = inter.getWatchPath().get(mCurrentPosition);
            pathToUpdate.setPositions(newPath.getPositions());
            pathToUpdate.setClosed(newPath.isClosed());
            Object[] tab = {inter.getId(), pathToUpdate};
            updatePathsForInter = new UpdatePathsForInterventionTask(this, EPathOperation.UPDATE);
            updatePathsForInter.execute(tab);

            // else, we add the new path
        } else {
            Object[] tab = {inter.getId(), newPath};
            updatePathsForInter = new UpdatePathsForInterventionTask(this, EPathOperation.CREATE);
            updatePathsForInter.execute(tab);
        }


        ((PlanZoneActivity)getActivity()).showProgress(true);
        // remove Click listener
        resetMapListener();

        // if we are in edition mode, we set the new area in the intervention we get back from the main activity
        if (editPath){
            // send to the database
            Area areaToUpdate = inter.getWatchArea().get(mCurrentPositionArea);
            areaToUpdate.setPositions(polygonPoints);
            //TODO maybe something to do here before sending
            Object[] tab = {inter.getId(), areaToUpdate};
            updateAreaTask = new UpdateAreaTask(this, EPathOperation.UPDATE);
            updateAreaTask.execute(tab);

            // else, we add the new area
        } else {
            //TODO set newArea during creation
            Object[] tab = {inter.getId(), newArea};
            updateAreaTask = new UpdateAreaTask(this, EPathOperation.CREATE);
            updateAreaTask.execute(tab);
        }
    }

    public void checkIfEnableCreateButton() {
        if (markers.size() > 0){
            PlanZoneActivity pl = (PlanZoneActivity) getActivity();
            pl.enableCreatePathButton();
            pl.enableCreateAreaButton();

        }
    }
    
}


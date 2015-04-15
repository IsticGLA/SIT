package istic.gla.groupeb.flerjeco.agent.planZone;

import android.app.ActionBar;
import android.app.FragmentManager;
import android.content.Intent;
import android.os.Bundle;
import android.support.v4.app.FragmentActivity;
import android.support.v4.app.FragmentTransaction;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.LinearLayout;


import java.util.ArrayList;
import java.util.List;

import entity.Intervention;
import entity.Path;
import entity.Position;
import entity.Resource;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.agent.intervention.AgentInterventionActivity;
import util.ResourceCategory;
import util.ResourceRole;
import util.State;

public class PlanZoneActivity extends FragmentActivity implements DroneListFragment.OnResourceSelectedListener, ActionBar.TabListener {

    private static final String TAG = PlanZoneActivity.class.getSimpleName();
    private Intervention intervention;
    private int position=0;
    private boolean editionMode = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Bundle extras = getIntent().getExtras();
        if (extras != null) {
            Object objects = (Object) extras.getSerializable("intervention");
            intervention = (Intervention) objects;

        }

        // Temporary intervention for test
        intervention = new Intervention("Test", 4, 48.1120404, -1.61111);
        List<Resource> resources = new ArrayList<>();
        //resources.add(new Resource("Drone1", State.validated, ResourceRole.otherwise, ResourceCategory.drone, 48.117749, -1.677297));
        intervention.setResources(resources);
        List<Path> paths = new ArrayList<>();
        Path p = new Path();
        List<Position> positionList = new ArrayList<>();
        positionList.add(new Position(48.117749, -1.677297));
        positionList.add(new Position(48.127749, -1.699297));
        positionList.add(new Position(48.227749, -1.707297));
        p.setPositions(positionList);
        paths.add(p);
        intervention.setWatchPath(paths);
        intervention.setId(10l);

        setContentView(R.layout.activity_plan_zone);

        // Check whether the activity is using the layout version with
        // the fragment_container FrameLayout. If so, we must add the first fragment
        if (findViewById(R.id.fragment_container) != null) {

            // However, if we're being restored from a previous state,
            // then we don't need to do anything and should return or else
            // we could end up with overlapping fragments.
            if (savedInstanceState != null) {
                return;
            }

            // Create an instance of ExampleFragment
            DroneListFragment firstFragment = new DroneListFragment();

            // In case this activity was started with special instructions from an Intent,
            // pass the Intent's extras to the fragment as arguments
            firstFragment.setArguments(getIntent().getExtras());

            // Add the fragment to the 'fragment_container' FrameLayout
            getSupportFragmentManager().beginTransaction()
                    .add(R.id.fragment_container, firstFragment).commit();
        }

        final ActionBar actionBar = getActionBar();

        // Specify that tabs should be displayed in the action bar.
        actionBar.setNavigationMode(ActionBar.NAVIGATION_MODE_TABS);

        // Add 2 tabs, specifying the tab's text and TabListener
        ActionBar.Tab tabInter = actionBar.newTab();
        tabInter.setText("Intervention");
        tabInter.setTabListener(this);

        ActionBar.Tab tabDrone = actionBar.newTab();
        tabDrone.setText("Drone");
        tabDrone.setTabListener(this);
        actionBar.addTab(tabDrone);
        actionBar.addTab(tabInter, 0, false);
    }


    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_plan_zone, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    @Override
    public void onResourceSelected(int position) {
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);

        if (mapFragment != null) {
            // If article frag is available, we're in two-pane layout...

            //save the current position
            this.position = position;

            Log.i("EDITPATH", "SET TO TRUE ONRESOURCESELECTED");
            mapFragment.editPath = true;

            editModeOn();

            // Call a method in the ArticleFragment to update its content
            mapFragment.updateMapView(position);
            mapFragment.addMapClickListener();

        } else {
            // If the frag is not available, we're in the one-pane layout and must swap frags...

            // Create fragment and give it an argument for the selected article
            mapFragment = new PlanZoneMapFragment();
            Bundle args = new Bundle();
            args.putInt(PlanZoneMapFragment.ARG_POSITION, position);
            mapFragment.setArguments(args);
            FragmentTransaction transaction = getSupportFragmentManager().beginTransaction();

            // Replace whatever is in the fragment_container view with this fragment,
            // and add the transaction to the back stack so the user can navigate back
            transaction.replace(R.id.fragment_container, mapFragment);
            transaction.addToBackStack(null);

            // Commit the transaction
            transaction.commit();
        }
    }

    public List<Resource> getResourceEntities() {
        return intervention.getResources();
    }

    public List<Path> getPaths() {
        return intervention.getWatchPath();
    }

    public Intervention getIntervention(){
        return intervention;
    }

    public void createPath(View v){
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);

        if (!editionMode) {
            Log.i(TAG, "Mode d'édition du trajet");
            editModeOn();
            checkCloseBox(false);
            // begin the creation of the new path (add event on Google Map)
            mapFragment.createPath();

        } else  {
            // Reset button and checkbox
            editModeOff();
            // send the newPath on databse
            mapFragment.sendPath();
        }
    }

    public void closePath(View v){
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);
        mapFragment.closePath();
    }

    public void removeLastPoint(View v){
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);
        mapFragment.removeLastPoint();
    }

    public void cancel(View v){
        editModeOff();
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);
        mapFragment.resetMapListener();
        if (mapFragment.editPath) {
            mapFragment.updateMapView(position);
            Log.i("EDITPATH", "SET TO FALSE CANCEL");
            mapFragment.editPath = false;
        } else {
            mapFragment.clearGoogleMap();
        }
    }

    public void removePath(View v){
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);
        mapFragment.removePath();
    }

    public void editModeOff(){
        Button button = (Button) findViewById(R.id.buttonCreatePath);
        Button cancel = (Button) findViewById(R.id.buttonCancel);
        Button removeLast = (Button) findViewById(R.id.buttonRemoveLastPoint);
        Button removePath = (Button) findViewById(R.id.buttonRemove);
        CheckBox checkBox = (CheckBox) findViewById(R.id.checkbox_closed_path);
        button.setText(getString(R.string.create_path));
        checkBox.setChecked(false);
        checkBox.setVisibility(View.GONE);
        removeLast.setVisibility(View.GONE);
        cancel.setVisibility(View.GONE);
        removePath.setVisibility(View.GONE);
        editionMode = false;
    }

    public void editModeOn(){
        editionMode = true;

        Button button = (Button) findViewById(R.id.buttonCreatePath);
        Button cancel = (Button) findViewById(R.id.buttonCancel);
        Button removePath = (Button) findViewById(R.id.buttonRemove);
        Button removeLast = (Button) findViewById(R.id.buttonRemoveLastPoint);
        CheckBox checkBox = (CheckBox) findViewById(R.id.checkbox_closed_path);
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);

        // unselect on the listView
        DroneListFragment droneListFragment = (DroneListFragment)
                getSupportFragmentManager().findFragmentById(R.id.resources_fragment);
        droneListFragment.unCheckedListView();

        // show edit mode buttons
        button.setText(getString(R.string.finish_edition));
        cancel.setVisibility(View.VISIBLE);
        removeLast.setVisibility(View.VISIBLE);
        checkBox.setVisibility(View.VISIBLE);

        if (mapFragment.editPath){
            removePath.setVisibility(View.VISIBLE);
        }
    }

    public void checkCloseBox(boolean b){
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);
        CheckBox checkBox = (CheckBox) findViewById(R.id.checkbox_closed_path);

        checkBox.setChecked(b);
    }

    public void refreshList(Intervention intervention){
        this.intervention = intervention;
        DroneListFragment droneListFragment = (DroneListFragment)
                getSupportFragmentManager().findFragmentById(R.id.resources_fragment);
        droneListFragment.refresh(intervention);
    }

    @Override
    public void onTabSelected(ActionBar.Tab tab, android.app.FragmentTransaction ft) {
        if(tab.getText().toString().equals("Intervention")) {
            Intent intent = new Intent(PlanZoneActivity.this, AgentInterventionActivity.class);
            Bundle bundle = new Bundle();
            bundle.putSerializable("intervention", intervention);
            intent.putExtras(bundle);
            startActivity(intent);
            finish();
        }
    }

    @Override
    public void onTabUnselected(ActionBar.Tab tab, android.app.FragmentTransaction ft) {
        if(tab.getText().toString().equals("Drone")) {
            finish();
        }
    }

    @Override
    public void onTabReselected(ActionBar.Tab tab, android.app.FragmentTransaction ft) {

    }
}

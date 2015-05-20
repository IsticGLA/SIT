package istic.gla.groupeb.flerjeco.agent.planZone;

import android.app.ActionBar;
import android.app.ProgressDialog;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.support.v4.app.FragmentActivity;
import android.support.v4.app.FragmentTransaction;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;

import java.util.Collections;
import java.util.Comparator;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Path;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.agent.intervention.AgentInterventionActivity;
import istic.gla.groupeb.flerjeco.agent.table.TableActivity;
import istic.gla.groupeb.flerjeco.login.LoginActivity;
import istic.gla.groupeb.flerjeco.springRest.GetInterventionTask;
import istic.gla.groupeb.flerjeco.springRest.IInterventionActivity;
import istic.gla.groupeb.flerjeco.synch.DisplaySynch;
import istic.gla.groupeb.flerjeco.synch.ISynchTool;
import istic.gla.groupeb.flerjeco.synch.IntentWraper;

public class PlanZoneActivity extends FragmentActivity implements DroneListFragment.OnResourceSelectedListener, ActionBar.TabListener, ISynchTool, IInterventionActivity {

    private static final String TAG = PlanZoneActivity.class.getSimpleName();

    // current intervention
    private Intervention intervention; /*= new Intervention("Test", 2, 48.399, -1.6554);*/

    // current position of the path in the ListView DroneListFragment
    private int position=0;

    // save of the edition mode
    private boolean editionMode = false;

    private ProgressDialog progressDialog;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // Get the Bundle from the intent
        Bundle extras = getIntent().getExtras();

        // if we got an extras, we set the intervention to the value of the extras
        if (extras != null){
            Log.i(TAG, "Get the intervention from the bundle");
            intervention = (Intervention) extras.getSerializable("intervention");
        }

        // Set the content view with the activity_plan_zone layout
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

        // Add 3 tabs, specifying the tab's text and TabListener
        ActionBar.Tab tabInter = actionBar.newTab();
        tabInter.setText("Intervention");
        tabInter.setTabListener(this);

        ActionBar.Tab tabTableau = actionBar.newTab();
        tabTableau.setText("Tableau");
        tabTableau.setTabListener(this);

        ActionBar.Tab tabDrone = actionBar.newTab();
        tabDrone.setText("Drone");
        tabDrone.setTabListener(this);



        actionBar.addTab(tabInter, 0, false);
        actionBar.addTab(tabTableau, 1, false);
        actionBar.addTab(tabDrone, 2, true);
    }


    // Action Menu for Logout
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu items for use in the action bar
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.menu_logout, menu);
        return super.onCreateOptionsMenu(menu);
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.menu_logout:
                Intent intent = new Intent(PlanZoneActivity.this, LoginActivity.class);
                intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP | Intent.FLAG_ACTIVITY_NEW_TASK);
                startActivity(intent);
                finish();
            default:
                return super.onOptionsItemSelected(item);
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        if(intervention != null) {
            DisplaySynch displaySynch = new DisplaySynch() {
                @Override
                public void ctrlDisplay() {
                    refresh();
                }
            };
            String url = "notify/intervention/" + intervention.getId();
            IntentWraper.startService(url, displaySynch);
        }
    }

    @Override
    public void onResourceSelected(int position) {
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);

        // If article frag is available, we're in two-pane layout...
        if (mapFragment != null) {

            //save the current position
            this.position = position;

            // Set edition Mode on
            mapFragment.editPath = true;
            editModeOn();

            // Call a method in the ArticleFragment to update its content
            mapFragment.updateMapView(position);
            mapFragment.addMapClickListener();

        // If the frag is not available, we're in the one-pane layout and must swap frags...
        } else {

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

    /**
     * get the intervention set by the init Bundle on the launch of the activity
     * @return the Intervention
     */
    public Intervention getIntervention(){
        return intervention;
    }

    public boolean isEditionMode() {
        return editionMode;
    }

    /**
     * create a new path on the current activity when we click on create button
     * @param v the view of the activity
     */
    public void createPath(View v){
	PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);

        // if we are in edition mode, initi of the edit buttons
        if (!editionMode) {
            Log.i(TAG, "Mode d'édition du trajet");
            editModeOn();
            checkCloseBox(false);
            // begin the creation of the new path (add event on Google Map)
            mapFragment.createPath();
        // else, hide edit buttons and send the path on the database
        } else  {
            // Reset button and checkbox
            editModeOff();
            // send the newPath on databse
            mapFragment.sendPath();
        }
    }

    /**
     * close/unclose the current path when we click on checkbox
     * @param v the view of the activity
     */
    public void closePath(View v){
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);
        mapFragment.closePath();
    }

    /**
     * remove last point on the current path when we click on the remove_last_point button
     * @param v the view of the activity
     */
    public void removeLastPoint(View v){
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);
        mapFragment.removeLastPoint();
    }

    /**
     * cancel the current edition on the current path when we click on the cancel button
     * @param v the view of the activity
     */
    public void cancel(View v){
        // hide edit buttons
        editModeOff();
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);
        // remove the click Listener on the Google Map
        mapFragment.resetMapListener();
        // if we are in edition mode, we clear the path we are updating
        if (mapFragment.editPath) {
            mapFragment.updateMapView(position);
            mapFragment.editPath = false;
        // else, we clear the Google Map
        } else {
            mapFragment.clearGoogleMap();
        }
    }

    /**
     * remove the current path
     * @param v the view of the activity
     */
    public void removePath(View v){
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);
        mapFragment.removePath();
    }

    /**
     * clear the edit button on the ListView fragment
     */
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

    /**
     * show the edit button on the ListView fragment
     */
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

        // if we are in edition mode, we show the remove path button
        if (mapFragment.editPath){
            removePath.setVisibility(View.VISIBLE);
        }
    }

    /**
     * Check/uncheck the checkBox
     * @param b if b check else uncheck
     */
    public void checkCloseBox(boolean b){
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);
        CheckBox checkBox = (CheckBox) findViewById(R.id.checkbox_closed_path);
        checkBox.setChecked(b);
    }

    public void showProgress(boolean isProgress) {
        Log.i(TAG, "showProgress start");
        if(progressDialog == null) {
            Log.i(TAG, "progressDialog null");
            progressDialog = new ProgressDialog(PlanZoneActivity.this);
            progressDialog.setTitle("Chargement");
        }
        if(isProgress) {
            progressDialog.show();
        } else {
            progressDialog.dismiss();
        }
    }

    /**
     * refresh the list in the ListView fragment
     * @param intervention intervention updated
     */
    public void refreshList(Intervention intervention){
        Collections.sort(intervention.getWatchPath(), new Comparator<Path>() {
            @Override
            public int compare(Path lhs, Path rhs) {
                return Long.valueOf(lhs.getIdPath()).compareTo(Long.valueOf(rhs.getIdPath()));
            }
        });
        this.intervention = intervention;
        DroneListFragment droneListFragment = (DroneListFragment)
                getSupportFragmentManager().findFragmentById(R.id.resources_fragment);
        if(droneListFragment != null) {
            droneListFragment.refresh(intervention);
        }
    }

    public void checkListView(int position){
        DroneListFragment droneListFragment = (DroneListFragment)
                getSupportFragmentManager().findFragmentById(R.id.resources_fragment);
        droneListFragment.checkListView(position);
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
        }else if(tab.getText().toString().equals("Tableau")){
            Intent intent = new Intent(PlanZoneActivity.this, TableActivity.class);
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

    @Override
    public void updateIntervention(Intervention newIntervention) {
        Intervention oldIntervention = intervention;
        refreshList(newIntervention);
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);
        if(mapFragment != null) {
            mapFragment.refreshMapAfterSynchro(newIntervention, oldIntervention);
        }
    }

    @Override
    public Context getContext() {
        return getApplicationContext();
    }

    @Override
    public void refresh() {
        if (null != intervention) {
            new GetInterventionTask(this, intervention.getId()).execute();
        }
    }
}

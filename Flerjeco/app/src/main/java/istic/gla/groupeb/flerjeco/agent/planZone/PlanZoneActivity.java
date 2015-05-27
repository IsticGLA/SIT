package istic.gla.groupeb.flerjeco.agent.planZone;

import android.app.ProgressDialog;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
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
import istic.gla.groupeb.flerjeco.TabbedActivity;
import istic.gla.groupeb.flerjeco.login.LoginActivity;
import istic.gla.groupeb.flerjeco.springRest.GetInterventionTask;
import istic.gla.groupeb.flerjeco.springRest.IInterventionActivity;
import istic.gla.groupeb.flerjeco.synch.DisplaySynch;
import istic.gla.groupeb.flerjeco.synch.ISynchTool;
import istic.gla.groupeb.flerjeco.synch.IntentWraper;

public class PlanZoneActivity extends TabbedActivity implements DroneListFragment.OnResourceSelectedListener, ISynchTool, IInterventionActivity {

    private static final String TAG = PlanZoneActivity.class.getSimpleName();

    // current position of the path in the ListView DroneListFragment
    private int position=0;

    // save of the edition mode
    private boolean editionMode = false;

    private ProgressDialog progressDialog;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        //opening transition animations
        overridePendingTransition(0, android.R.anim.fade_out);

        intervention = new Intervention();

        Bundle extras = getIntent().getExtras();

        if (extras != null){
            Log.i(TAG, "getExtras not null");
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
            editModeOn(ECreationType.AREA);

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
            editModeOn(ECreationType.PATH);
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
     */
    public void removeLastPoint(ECreationType creationType){
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);
        mapFragment.removeLastPoint(creationType);
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
     * @param creationType the view of the activity
     */
    public void removePath(ECreationType creationType){
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);
        mapFragment.removePath(creationType);
    }

    /**
     * clear the edit button on the ListView fragment
     */
    public void editModeOff(){
        Button buttonP = (Button) findViewById(R.id.buttonCreatePath);
        Button buttonA = (Button) findViewById(R.id.buttonCreateArea);
        Button cancel = (Button) findViewById(R.id.buttonCancel);
        Button removeLast = (Button) findViewById(R.id.buttonRemoveLastPoint);
        Button removePath = (Button) findViewById(R.id.buttonRemove);
        CheckBox checkBox = (CheckBox) findViewById(R.id.checkbox_closed_path);
        buttonP.setText(getString(R.string.create_path));
        buttonA.setText(R.string.create_area);
        checkBox.setChecked(false);
        checkBox.setVisibility(View.GONE);
        removeLast.setVisibility(View.GONE);
        cancel.setVisibility(View.GONE);
        removePath.setVisibility(View.GONE);
        buttonP.setVisibility(View.VISIBLE);
        buttonA.setVisibility(View.VISIBLE);
        editionMode = false;
    }

    /**
     * show the edit button on the ListView fragment
     * @param creationType
     */
    public void editModeOn(final ECreationType creationType){
        editionMode = true;

        Button buttonP = (Button) findViewById(R.id.buttonCreatePath);
        Button buttonA = (Button) findViewById(R.id.buttonCreateArea);
        Button cancel = (Button) findViewById(R.id.buttonCancel);
        Button removePath = (Button) findViewById(R.id.buttonRemove);
        final Button removeLast = (Button) findViewById(R.id.buttonRemoveLastPoint);
        CheckBox checkBox = (CheckBox) findViewById(R.id.checkbox_closed_path);
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);

        // unselect on the listView
        DroneListFragment droneListFragment = (DroneListFragment)
                getSupportFragmentManager().findFragmentById(R.id.resources_fragment);
        droneListFragment.unCheckedListView();

        // show edit mode buttons
        buttonP.setText(getString(R.string.finish_edition));
        buttonA.setText(getString(R.string.finish_edition));
        cancel.setVisibility(View.VISIBLE);
        removeLast.setVisibility(View.VISIBLE);
        checkBox.setVisibility(View.VISIBLE);
        if(creationType == ECreationType.AREA) {
            buttonP.setVisibility(View.GONE);
            buttonA.setVisibility(View.VISIBLE);
        } else if(creationType == ECreationType.PATH) {
            buttonA.setVisibility(View.GONE);
            buttonP.setVisibility(View.VISIBLE);
        }

        removeLast.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                removeLastPoint(creationType);
            }
        });

        removePath.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                removePath(creationType);
            }
        });

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
            progressDialog.setCancelable(false);
            progressDialog.setCanceledOnTouchOutside(false);
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

    @Override
    protected void onPause() {
        super.onPause();
        //closing transition animations
        overridePendingTransition(R.anim.activity_open_scale,R.anim.activity_close_translate);
        IntentWraper.stopService();
    }

    /**
     * create a new path on the current activity when we click on create button
     * @param v the view of the activity
     */
    public void createArea(View v){
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);

        // if we are in edition mode, initi of the edit buttons
        if (!editionMode) {
            Log.i(TAG, "Mode d'édition de la zone");
            editModeOn(ECreationType.AREA);
            checkCloseBox(false);
            // begin the creation of the new path (add event on Google Map)
            mapFragment.createArea();
            // else, hide edit buttons and send the path on the database
        } else  {
            // Reset button and checkbox
            editModeOffArea();
            // send the newPath on databse
            mapFragment.sendPath();
        }
    }

    /**
     * clear the edit button on the ListView fragment
     */
    public void editModeOffArea(){
        Button buttonP = (Button) findViewById(R.id.buttonCreatePath);
        Button buttonA = (Button) findViewById(R.id.buttonCreateArea);
        Button cancel = (Button) findViewById(R.id.buttonCancel);
        Button removeLast = (Button) findViewById(R.id.buttonRemoveLastPoint);
        Button removePath = (Button) findViewById(R.id.buttonRemove);
        CheckBox checkBox = (CheckBox) findViewById(R.id.checkbox_closed_path);
        buttonP.setText(getString(R.string.create_path));
        checkBox.setChecked(false);
        checkBox.setVisibility(View.GONE);
        removeLast.setVisibility(View.GONE);
        cancel.setVisibility(View.GONE);
        removePath.setVisibility(View.GONE);
        buttonA.setVisibility(View.VISIBLE);
        editionMode = false;
    }
}

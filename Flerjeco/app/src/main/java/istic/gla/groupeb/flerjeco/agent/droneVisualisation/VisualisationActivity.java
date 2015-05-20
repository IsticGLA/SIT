package istic.gla.groupeb.flerjeco.agent.droneVisualisation;

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

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.agent.AgentTabbedActivity;
import istic.gla.groupeb.flerjeco.agent.intervention.AgentInterventionActivity;
import istic.gla.groupeb.flerjeco.agent.planZone.DroneListFragment;
import istic.gla.groupeb.flerjeco.agent.planZone.PlanZoneActivity;
import istic.gla.groupeb.flerjeco.agent.table.TableActivity;
import istic.gla.groupeb.flerjeco.login.LoginActivity;
import istic.gla.groupeb.flerjeco.springRest.GetInterventionTask;
import istic.gla.groupeb.flerjeco.springRest.IInterventionActivity;
import istic.gla.groupeb.flerjeco.synch.DisplaySynch;
import istic.gla.groupeb.flerjeco.synch.ISynchTool;
import istic.gla.groupeb.flerjeco.synch.IntentWraper;

public class VisualisationActivity extends AgentTabbedActivity implements ISynchTool, IInterventionActivity {

    private static final String TAG = VisualisationActivity.class.getSimpleName();

    // current intervention
    private Intervention intervention; /*= new Intervention("Test", 2, 48.399, -1.6554);*/

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

        // Add 4 tabs, specifying the tab's text and TabListener
        ActionBar.Tab tabInter = actionBar.newTab();
        tabInter.setText("Intervention");
        tabInter.setTabListener(this);

        ActionBar.Tab tabTableau = actionBar.newTab();
        tabTableau.setText("Tableau");
        tabTableau.setTabListener(this);

        ActionBar.Tab tabDrone = actionBar.newTab();
        tabDrone.setText("Drone");
        tabDrone.setTabListener(this);

        ActionBar.Tab tabVisu = actionBar.newTab();
        tabDrone.setText("Visualisation");
        tabDrone.setTabListener(this);


        actionBar.addTab(tabInter, 0, false);
        actionBar.addTab(tabTableau, 1, false);
        actionBar.addTab(tabDrone, 2, false);
        actionBar.addTab(tabVisu, 3, true);
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
                Intent intent = new Intent(VisualisationActivity.this, LoginActivity.class);
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

    /**
     * get the intervention set by the init Bundle on the launch of the activity
     * @return the Intervention
     */
    public Intervention getIntervention(){
        return intervention;
    }

    /**
     * refresh the list in the ListView fragment
     * @param intervention intervention updated
     */
    public void refreshList(Intervention intervention){
        this.intervention = intervention;
        DroneListFragment droneListFragment = (DroneListFragment)
                getSupportFragmentManager().findFragmentById(R.id.resources_fragment);
        droneListFragment.refresh(intervention);
    }

    @Override
    public void updateIntervention(Intervention intervention) {
        refreshList(intervention);
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

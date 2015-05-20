package istic.gla.groupeb.flerjeco.agent.tableau;

import android.app.ActionBar;
import android.content.Context;
import android.content.Intent;
import android.net.Uri;
import android.support.v4.app.FragmentActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;

import entity.Intervention;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.agent.intervention.AgentInterventionActivity;
import istic.gla.groupeb.flerjeco.agent.planZone.PlanZoneActivity;
import istic.gla.groupeb.flerjeco.springRest.GetInterventionTask;
import istic.gla.groupeb.flerjeco.springRest.IInterventionActivity;
import istic.gla.groupeb.flerjeco.synch.DisplaySynch;
import istic.gla.groupeb.flerjeco.synch.ISynchTool;
import istic.gla.groupeb.flerjeco.synch.IntentWraper;

public class TableauActivity extends FragmentActivity implements  ActionBar.TabListener, ISynchTool, IInterventionActivity, TableauFragment.OnFragmentInteractionListener{

    private static final String TAG = TableauActivity.class.getSimpleName();

    protected Intervention intervention;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_tableau);

        intervention = new Intervention();

        Bundle extras = getIntent().getExtras();

        if (extras != null){
            Log.i(TAG, "getExtras not null");
            intervention = (Intervention) extras.getSerializable("intervention");
        }


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
            TableauFragment firstFragment = new TableauFragment();

            // In case this activity was started with special instructions from an Intent,
            // pass the Intent's extras to the fragment as arguments
            firstFragment.setArguments(getIntent().getExtras());

            // Add the fragment to the 'fragment_container' FrameLayout
            getSupportFragmentManager().beginTransaction().add(R.id.fragment_container, firstFragment).commit();

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
        actionBar.addTab(tabTableau,1,true);
        actionBar.addTab(tabDrone,2,false);
    }


    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_tableau, menu);
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
    public void onTabSelected(ActionBar.Tab tab, android.app.FragmentTransaction ft) {
        Intent intent = null;
        if(tab.getText().toString().equals("Drone")) {
            intent = new Intent(TableauActivity.this, PlanZoneActivity.class);
            Bundle bundle = new Bundle();
            bundle.putSerializable("intervention", intervention);
            intent.putExtras(bundle);
            startActivity(intent);
            finish();
        }else if(tab.getText().toString().equals("Intervention")){
            intent = new Intent(TableauActivity.this, AgentInterventionActivity.class);
            Bundle bundle = new Bundle();
            bundle.putSerializable("intervention", intervention);
            intent.putExtras(bundle);
            startActivity(intent);
            finish();
        }
    }

    @Override
    public void onTabUnselected(ActionBar.Tab tab, android.app.FragmentTransaction ft) {
        if(tab.getText().toString().equals("Tableau")) {
            finish();
        }
    }

    @Override
    public void onTabReselected(ActionBar.Tab tab, android.app.FragmentTransaction ft) {
    }


    @Override
    public Context getContext() {
        return getApplicationContext();
    }

    /**
     * Update intervention
     * @param intervention
     */
    public void updateIntervention(Intervention intervention) {
        Log.i(TAG, "updateIntervention");
        this.intervention = intervention;
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
    public void refresh(){
        if (null != intervention) {
            new GetInterventionTask(this, intervention.getId()).execute();
        }
    }
    @Override
    protected void onStop() {
        super.onStop();
    }

    @Override
    protected void onPause() {
        super.onPause();
        IntentWraper.stopService();
    }

    @Override
    public void onFragmentInteraction(Uri uri) {

    }
}

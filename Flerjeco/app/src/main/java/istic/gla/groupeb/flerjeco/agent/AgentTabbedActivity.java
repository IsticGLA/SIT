package istic.gla.groupeb.flerjeco.agent;

import android.app.ActionBar;
import android.content.Intent;
import android.os.Bundle;
import android.support.v4.app.FragmentActivity;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.agent.droneVisualisation.VisualisationActivity;
import istic.gla.groupeb.flerjeco.agent.intervention.AgentInterventionActivity;
import istic.gla.groupeb.flerjeco.agent.planZone.PlanZoneActivity;
import istic.gla.groupeb.flerjeco.agent.table.TableActivity;

/**
 * Created by jules on 20/05/15.
 */
public abstract class AgentTabbedActivity extends FragmentActivity implements ActionBar.TabListener {

    private String tabName;
    protected Intervention intervention;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        intervention = new Intervention();

        Bundle extras = getIntent().getExtras();

        if (extras != null){
            intervention = (Intervention) extras.getSerializable("intervention");
        }

        final ActionBar actionBar = getActionBar();

        // Specify that tabs should be displayed in the action bar.
        actionBar.setNavigationMode(ActionBar.NAVIGATION_MODE_TABS);

        // Add 4 tabs, specifying the tab's text and TabListener
        ActionBar.Tab tabInter = actionBar.newTab();
        tabInter.setText(R.string.intervention);
        tabInter.setTabListener(this);

        ActionBar.Tab tabTableau = actionBar.newTab();
        tabTableau.setText(R.string.table);
        tabTableau.setTabListener(this);

        ActionBar.Tab tabDrone = actionBar.newTab();
        tabDrone.setText(R.string.drones);
        tabDrone.setTabListener(this);

        ActionBar.Tab tabVisu = actionBar.newTab();
        tabDrone.setText(R.string.visualisation);
        tabDrone.setTabListener(this);

        if (this.getClass() == TableActivity.class) {
            tabName = getResources().getString(R.string.table);
            actionBar.addTab(tabTableau, 1, true);
        } else if (this.getClass() == VisualisationActivity.class) {
            tabName = getResources().getString(R.string.visualisation);
            actionBar.addTab(tabVisu, 3, true);
        } else if (this.getClass() == PlanZoneActivity.class) {
            tabName = getResources().getString(R.string.drones);
            actionBar.addTab(tabDrone, 2, true);
        } else if (this.getClass() == AgentInterventionActivity.class) {
            tabName = getResources().getString(R.string.intervention);
            actionBar.addTab(tabInter, 0, true);
        }
    }

    @Override
    public void onTabSelected(ActionBar.Tab tab, android.app.FragmentTransaction ft) {
        if(tab.getText().toString().equals(R.string.intervention)) {
            Intent intent = new Intent(AgentTabbedActivity.this, AgentInterventionActivity.class);
            Bundle bundle = new Bundle();
            bundle.putSerializable("intervention", intervention);
            intent.putExtras(bundle);
            startActivity(intent);
            finish();
        }else if(tab.getText().toString().equals(R.string.table)){
            Intent intent = new Intent(AgentTabbedActivity.this, TableActivity.class);
            Bundle bundle = new Bundle();
            bundle.putSerializable("intervention", intervention);
            intent.putExtras(bundle);
            startActivity(intent);
            finish();
        } else if(tab.getText().toString().equals(R.string.drones)){
            Intent intent = new Intent(AgentTabbedActivity.this, PlanZoneActivity.class);
            Bundle bundle = new Bundle();
            bundle.putSerializable("intervention", intervention);
            intent.putExtras(bundle);
            startActivity(intent);
            finish();
        } else if(tab.getText().toString().equals(R.string.visualisation)){
            Intent intent = new Intent(AgentTabbedActivity.this, VisualisationActivity.class);
            Bundle bundle = new Bundle();
            bundle.putSerializable("intervention", intervention);
            intent.putExtras(bundle);
            startActivity(intent);
            finish();
        }
    }

    @Override
    public void onTabUnselected(ActionBar.Tab tab, android.app.FragmentTransaction ft) {
        if(tab.getText().toString().equals(tabName)) {
            finish();
        }
    }

    @Override
    public void onTabReselected(ActionBar.Tab tab, android.app.FragmentTransaction ft) {

    }

    public Intervention getIntervention() {
        return intervention;
    }
}

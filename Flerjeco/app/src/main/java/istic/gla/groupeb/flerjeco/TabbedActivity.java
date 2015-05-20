package istic.gla.groupeb.flerjeco;

import android.app.ActionBar;
import android.content.Intent;
import android.os.Bundle;
import android.support.v4.app.FragmentActivity;

import java.util.ArrayList;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.agent.droneVisualisation.VisualisationActivity;
import istic.gla.groupeb.flerjeco.agent.intervention.AgentInterventionActivity;
import istic.gla.groupeb.flerjeco.agent.planZone.PlanZoneActivity;
import istic.gla.groupeb.flerjeco.agent.table.TableActivity;

/**
 * Created by jules on 20/05/15.
 */
public abstract class TabbedActivity extends FragmentActivity implements ActionBar.TabListener {

    private String tabName;
    protected Intervention intervention;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        ArrayList<Integer> tabs = null;

        Bundle extras = getIntent().getExtras();

        if (extras != null){
            tabs = extras.getIntegerArrayList("tabs");
        }

        final ActionBar actionBar = getActionBar();

        // Specify that tabs should be displayed in the action bar.
        actionBar.setNavigationMode(ActionBar.NAVIGATION_MODE_TABS);

        if(tabs.contains(R.string.intervention)) {
            ActionBar.Tab tabInter = actionBar.newTab();
            tabInter.setText(R.string.intervention);
            tabInter.setTabListener(this);
            actionBar.addTab(tabInter, 0, this.getClass() == AgentInterventionActivity.class);
        }
        if(tabs.contains(R.string.table)) {
            ActionBar.Tab tabTable = actionBar.newTab();
            tabTable.setText(R.string.table);
            tabTable.setTabListener(this);
            actionBar.addTab(tabTable, 1, this.getClass() == TableActivity.class);
        }
        if(tabs.contains(R.string.drones)) {
            ActionBar.Tab tabDrone = actionBar.newTab();
            tabDrone.setText(R.string.drones);
            tabDrone.setTabListener(this);
            actionBar.addTab(tabDrone, 2, this.getClass() == PlanZoneActivity.class);
        }
        if(tabs.contains(R.string.visualisation)) {
            ActionBar.Tab tabVisu = actionBar.newTab();
            tabVisu.setText(R.string.visualisation);
            tabVisu.setTabListener(this);
            actionBar.addTab(tabVisu, 3, this.getClass() == VisualisationActivity.class);
        }

        if (this.getClass() == AgentInterventionActivity.class) {
            tabName = getResources().getString(R.string.intervention);
        } else if (this.getClass() == TableActivity.class) {
            tabName = getResources().getString(R.string.table);
        } else if (this.getClass() == PlanZoneActivity.class) {
            tabName = getResources().getString(R.string.drones);
        } else if (this.getClass() == VisualisationActivity.class) {
            tabName = getResources().getString(R.string.visualisation);
        }
    }

    @Override
    public void onTabSelected(ActionBar.Tab tab, android.app.FragmentTransaction ft) {
        if(tab.getText().toString().equals(getResources().getString(R.string.intervention)) && !tab.getText().toString().equals(tabName)) {
            Intent intent = new Intent(this, AgentInterventionActivity.class);
            Bundle bundle = new Bundle();
            bundle.putSerializable("intervention", intervention);
            intent.putExtras(bundle);
            startActivity(intent);
            finish();
        }else if(tab.getText().toString().equals(getResources().getString(R.string.table)) && !tab.getText().toString().equals(tabName)) {
            Intent intent = new Intent(this, TableActivity.class);
            Bundle bundle = new Bundle();
            bundle.putSerializable("intervention", intervention);
            intent.putExtras(bundle);
            startActivity(intent);
            finish();
        } else if(tab.getText().toString().equals(getResources().getString(R.string.drones)) && !tab.getText().toString().equals(tabName)) {
            Intent intent = new Intent(this, PlanZoneActivity.class);
            Bundle bundle = new Bundle();
            bundle.putSerializable("intervention", intervention);
            intent.putExtras(bundle);
            startActivity(intent);
            finish();
        } else if(tab.getText().toString().equals(getResources().getString(R.string.visualisation)) && !tab.getText().toString().equals(tabName)) {
            Intent intent = new Intent(this, VisualisationActivity.class);
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

package istic.gla.groupeb.flerjeco.agent.table;

import android.app.ActionBar;
import android.content.Context;
import android.content.Intent;
import android.net.Uri;
import android.support.v4.app.FragmentActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Gravity;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.widget.TableLayout;
import android.widget.TableRow;

import android.widget.TableRow.LayoutParams;
import android.widget.TextView;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.agent.intervention.AgentInterventionActivity;
import istic.gla.groupeb.flerjeco.agent.planZone.PlanZoneActivity;
import istic.gla.groupeb.flerjeco.login.LoginActivity;
import istic.gla.groupeb.flerjeco.springRest.GetInterventionTask;
import istic.gla.groupeb.flerjeco.springRest.IInterventionActivity;
import istic.gla.groupeb.flerjeco.synch.DisplaySynch;
import istic.gla.groupeb.flerjeco.synch.ISynchTool;
import istic.gla.groupeb.flerjeco.synch.IntentWraper;

public class TableActivity extends FragmentActivity implements  ActionBar.TabListener, ISynchTool, IInterventionActivity, TableFragment.OnFragmentInteractionListener{

    private static final String TAG = TableActivity.class.getSimpleName();

    protected Intervention intervention;
    private TableFragment firstFragment;
    private  TableLayout headerTable;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_tableau);

        intervention = new Intervention();


        Bundle extras = getIntent().getExtras();

        if (extras != null){
            Log.i(TAG, "getExtras not null");
            intervention = (Intervention) extras.getSerializable("intervention");

            DisplaySynch displaySynch = new DisplaySynch() {
                @Override
                public void ctrlDisplay() {
                    refresh();
                }
            };
            String url = "notify/intervention/"+intervention.getId();
            IntentWraper.startService(url, displaySynch);
        }

        //Init of header table
        headerTableInit();


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
            firstFragment = new TableFragment();

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

    public void headerTableInit(){
        headerTable = (TableLayout) findViewById(R.id.headerTable);

        // Recuperation du table layout sur lequel nous allons agir
        String[] moyen = getResources().getStringArray(R.array.resourceDateState);

        // On va calculer la largeur des colonnes en fonction de la marge de 10
        // On affiche l'enreg dans une ligne
        TableRow tableRow = new TableRow(this);
        headerTable.addView(tableRow,
                new LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT));
        headerTable.setBackgroundColor(getResources().getColor(R.color.grey));

        // On crée une ligne de x moyen colonnes
        tableRow.setLayoutParams(new LayoutParams(moyen.length));

        int i = 0;
        for (String resourceDateState : moyen) {
            TextView text = createTextView(false , i == moyen.length - 1);
            text.setText(resourceDateState);
            text.setGravity(Gravity.CENTER);
            text.setWidth(100);
            tableRow.addView(text, i++);
        }
    }
    private TextView createTextView(boolean endline, boolean endcolumn){
        TextView text = new TextView(this, null, R.style.frag2HeaderCol);
        int bottom = endline ? 1 : 0;
        int right = endcolumn ? 1 :0;
        LayoutParams params = new LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.MATCH_PARENT, 0.3f);
        params.setMargins(1, 1, right, bottom);
        text.setLayoutParams(params);
        text.setPadding(4, 4, 10, 4);
        text.setBackgroundColor(getResources().getColor(R.color.white));
        return text;
    }
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
                Intent intent = new Intent(TableActivity.this, LoginActivity.class);
                intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP | Intent.FLAG_ACTIVITY_NEW_TASK);
                startActivity(intent);
                finish();
            default:
                return super.onOptionsItemSelected(item);
        }
    }


    @Override
    public void onTabSelected(ActionBar.Tab tab, android.app.FragmentTransaction ft) {
        Intent intent = null;
        if(tab.getText().toString().equals("Drone")) {
            intent = new Intent(TableActivity.this, PlanZoneActivity.class);
            Bundle bundle = new Bundle();
            bundle.putSerializable("intervention", intervention);
            intent.putExtras(bundle);
            startActivity(intent);
            finish();
        }else if(tab.getText().toString().equals("Intervention")){
            intent = new Intent(TableActivity.this, AgentInterventionActivity.class);
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

        if (firstFragment != null){
            firstFragment.refresh();
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

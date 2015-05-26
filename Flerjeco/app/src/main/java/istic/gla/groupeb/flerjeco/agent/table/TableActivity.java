package istic.gla.groupeb.flerjeco.agent.table;

import android.content.Context;
import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.util.Log;
import android.view.Gravity;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.TableLayout;
import android.widget.TableRow;
import android.widget.TableRow.LayoutParams;
import android.widget.TextView;

import org.joda.time.format.DateTimeFormat;
import org.joda.time.format.DateTimeFormatter;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupeb.flerjeco.FlerjecoApplication;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.TabbedActivity;
import istic.gla.groupeb.flerjeco.login.LoginActivity;
import istic.gla.groupeb.flerjeco.springRest.GetInterventionTask;
import istic.gla.groupeb.flerjeco.springRest.IInterventionActivity;
import istic.gla.groupeb.flerjeco.synch.DisplaySynch;
import istic.gla.groupeb.flerjeco.synch.ISynchTool;
import istic.gla.groupeb.flerjeco.synch.IntentWraper;

public class TableActivity extends TabbedActivity implements ISynchTool, IInterventionActivity, TableFragment.OnFragmentInteractionListener{

    private static final String TAG = TableActivity.class.getSimpleName();

    private TableFragment firstFragment;
    private  TableLayout headerTable;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        //opening transition animations
        overridePendingTransition(0, android.R.anim.fade_out);

        setContentView(R.layout.activity_table);

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
            String url = "notify/intervention/" + intervention.getId();
            IntentWraper.startService(url, displaySynch);
        }

        //Init of header table
        headerTableInit();

        //Set Title of Activity
        FlerjecoApplication flerjecoApplication = FlerjecoApplication.getInstance();
        if(flerjecoApplication.isCodisUser()) {
            setTitle(R.string.activities_codis);
        } else {
            setTitle(R.string.activities_agent);
        }
        DateTimeFormatter dtf = DateTimeFormat.forPattern("dd/MM/yyyy HH:mm:ss");
        String creationDate = dtf.print(intervention.getCreationDate());
        findViewById(R.id.text_intervention_title).setVisibility(View.VISIBLE);
        if(intervention.getAddress() != null){
            ((TextView) findViewById(R.id.text_intervention_title)).setText(
                    String.format(getString(R.string.codis_intervention_title),
                            intervention.getName(),
                            intervention.getAddress(),
                            creationDate));
        } else{
            ((TextView) findViewById(R.id.text_intervention_title)).setText(
                    String.format(getString(R.string.codis_intervention_title_without_address),
                            intervention.getName(),
                            creationDate));
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
            firstFragment = new TableFragment();

            // In case this activity was started with special instructions from an Intent,
            // pass the Intent's extras to the fragment as arguments
            firstFragment.setArguments(getIntent().getExtras());

            // Add the fragment to the 'fragment_container' FrameLayout
            getSupportFragmentManager().beginTransaction().add(R.id.fragment_container, firstFragment).commit();
        }
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

        if (firstFragment != null && !firstFragment.isDetached()){
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
        //closing transition animations
        overridePendingTransition(R.anim.activity_open_scale,R.anim.activity_close_translate);
        IntentWraper.stopService();
    }


    @Override
    public void onFragmentInteraction(Uri uri) {

    }
}

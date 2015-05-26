package istic.gla.groupeb.flerjeco.agent.droneVisualisation;

import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.support.v4.app.FragmentManager;
import android.support.v4.app.FragmentTransaction;
import android.util.Log;
import android.view.KeyEvent;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.LinearLayout;

import com.google.android.gms.maps.model.LatLng;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.TabbedActivity;
import istic.gla.groupeb.flerjeco.login.LoginActivity;
import istic.gla.groupeb.flerjeco.springRest.GetInterventionTask;
import istic.gla.groupeb.flerjeco.springRest.IInterventionActivity;
import istic.gla.groupeb.flerjeco.synch.DisplaySynch;
import istic.gla.groupeb.flerjeco.synch.ISynchTool;
import istic.gla.groupeb.flerjeco.synch.IntentWraper;

public class VisualisationActivity extends TabbedActivity implements ISynchTool, IInterventionActivity {

    private static final String TAG = VisualisationActivity.class.getSimpleName();

    private VisualisationMapFragment mapFragment;
    private ImageSliderFragment imageSliderFragment;
    private VideoFragment droneVideoFragment;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        //opening transition animations
        overridePendingTransition(0, android.R.anim.fade_out);

        Bundle extras = getIntent().getExtras();

        if (extras != null){
            Log.i(TAG, "getExtras not null");
            intervention = (Intervention) extras.getSerializable("intervention");
        }

        // Set the content view with the activity_visualisation layout
        setContentView(R.layout.activity_visualisation);

        // Check whether the activity is using the layout version with
        // the fragment_container FrameLayout. If so, we must add the first fragment
        if (findViewById(R.id.fragment_container) != null) {

            // However, if we're being restored from a previous state,
            // then we don't need to do anything and should return or else
            // we could end up with overlapping fragments.
            if (savedInstanceState != null) {
                return;
            }
        }

        mapFragment = (VisualisationMapFragment)
                getSupportFragmentManager().findFragmentById(R.id.map_fragment);
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

    @Override
    public void updateIntervention(Intervention intervention) {
        this.intervention = intervention;
        if (mapFragment!= null) {
            mapFragment.updateMapView();
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
     * charge la galerie d'image pour cette position
     * @param position la position a charger
     */
    public void loadImageSlider(LatLng position){
        Log.i(TAG, "loading image slider");
        FragmentManager fragmentManager = getSupportFragmentManager();
        FragmentTransaction fragmentTransaction = fragmentManager.beginTransaction();
        if(imageSliderFragment != null){
            fragmentTransaction.remove(imageSliderFragment);
        }
        removeDroneVideo();
        imageSliderFragment = new ImageSliderFragment();
        Bundle bundle = new Bundle();
        bundle.putLong("interventionId", intervention.getId());
        bundle.putDouble("latitude", position.latitude);
        bundle.putDouble("longitude", position.longitude);
        imageSliderFragment.setArguments(bundle);
        fragmentTransaction.add(R.id.image_slider_fragment_container, imageSliderFragment);
        fragmentTransaction.commit();
        LinearLayout imageSliderContainer = (LinearLayout) findViewById(R.id.image_slider_fragment_container);
        imageSliderContainer.setVisibility(View.VISIBLE);
    }

    /**
     * charge le stream d'image a afficher pour le drone
     * @param droneLabel le label du drone
     */
    public void loadDroneStream(String droneLabel){
        Log.i(TAG, "loading drone video");
        FragmentManager fragmentManager = getSupportFragmentManager();
        FragmentTransaction fragmentTransaction = fragmentManager.beginTransaction();
        if(droneVideoFragment != null){
            fragmentTransaction.remove(droneVideoFragment);
        }
        removeImageSlider();
        droneVideoFragment = new VideoFragment();
        Bundle bundle = new Bundle();
        bundle.putString("droneLabel", droneLabel);
        droneVideoFragment.setArguments(bundle);
        fragmentTransaction.add(R.id.drone_video_fragment_container, droneVideoFragment);
        fragmentTransaction.commit();
        LinearLayout droneVideoContainer = (LinearLayout) findViewById(R.id.drone_video_fragment_container);
        droneVideoContainer.setVisibility(View.VISIBLE);
    }

    /**
     * retire le fragment d'historique d'image
     */
    private void removeImageSlider(){
        //on retire le fragment
        if(imageSliderFragment != null) {
            FragmentManager fragmentManager = getSupportFragmentManager();
            FragmentTransaction fragmentTransaction = fragmentManager.beginTransaction();
            fragmentTransaction.remove(imageSliderFragment);
            fragmentTransaction.commit();
            LinearLayout imageSliderContainer = (LinearLayout) findViewById(R.id.image_slider_fragment_container);
            imageSliderContainer.setVisibility(View.GONE);
            imageSliderFragment = null;
        }
    }

    /**
     * retire le fragment de video de drone
     */
    private void removeDroneVideo(){
        //on retire le fragment
        if(droneVideoFragment != null) {
            FragmentManager fragmentManager = getSupportFragmentManager();
            FragmentTransaction fragmentTransaction = fragmentManager.beginTransaction();
            fragmentTransaction.remove(droneVideoFragment);
            fragmentTransaction.commit();
            LinearLayout droneVideoContainer = (LinearLayout) findViewById(R.id.drone_video_fragment_container);
            droneVideoContainer.setVisibility(View.GONE);
            droneVideoFragment = null;
        }
    }

    /**
     * capture la touche retour pour cacher les fragments
     * @param keyCode
     * @param event
     * @return
     */
    @Override
    public boolean onKeyDown(int keyCode, KeyEvent event) {
        if (keyCode == KeyEvent.KEYCODE_BACK && (imageSliderFragment != null || droneVideoFragment != null)) {
            if(imageSliderFragment != null){
                removeImageSlider();
            }
            if(droneVideoFragment != null){
                removeDroneVideo();
            }
            return true;
        }
        return super.onKeyDown(keyCode, event);
    }
}

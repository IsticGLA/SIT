package istic.gla.groupeb.flerjeco.agent.intervention;

import android.animation.Animator;
import android.animation.AnimatorListenerAdapter;
import android.annotation.TargetApi;
import android.content.DialogInterface;
import android.os.Build;
import android.os.Bundle;
import android.support.v4.app.DialogFragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Resource;
import istic.gla.groupb.nivimoju.util.State;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.springRest.IResourceActivity;
import istic.gla.groupeb.flerjeco.springRest.UpdateResourceTask;

/**
 * Created by corentin on 20/05/15.
 */
public class VehicleArrivedDialog extends DialogFragment implements IResourceActivity {

    private static final String TAG = VehicleArrivedDialog.class.getSimpleName();

    private View mProgressView;
    private View mVehicleFormView;
    private UpdateResourceTask updateResourceTask;

    private Resource mResource;
    private Long interventionId;

    public void setResource(Resource resource) {
        this.mResource = resource;
    }

    public void setInterventionId(Long interventionId) {
        this.interventionId = interventionId;
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View v = inflater.inflate(R.layout.fragment_resource_arrived, container, false);
        getDialog().setTitle(R.string.resource_arrived_dialog_title);

        mVehicleFormView = v.findViewById(R.id.vehicle_form);
        mProgressView = v.findViewById(R.id.vehicle_progress);

        final Button buttonArrived = (Button) v.findViewById(R.id.button_resource_arrived);
        buttonArrived.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                vehicleArrived(v);
            }
        });

        final Button buttonReleased = (Button) v.findViewById(R.id.button_resource_released);
        buttonReleased.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                vehicleReleased(v);
            }
        });

        return v;
    }

    public void vehicleArrived(View view) {
        Log.i(TAG, "vehicleArrived Start for : "+mResource.getLabel());
        showProgress(true);
        updateResourceTask = new UpdateResourceTask(this);
        updateResourceTask.execute(
                "" + interventionId,
                "" + mResource.getIdRes(),
                State.arrived.name());
    }

    public void vehicleReleased(View view) {
        Log.i(TAG, "vehicleReleased Start for : "+mResource.getLabel());
        showProgress(true);
        updateResourceTask = new UpdateResourceTask(this);
        updateResourceTask.execute(
                "" + interventionId,
                "" + mResource.getIdRes(),
                State.free.name());
    }

    @Override
    public void onCancel(DialogInterface dialog) {
        super.onCancel(dialog);
        if(updateResourceTask != null) {
            updateResourceTask.cancel(true);
        }
    }

    /**
     * Shows the progress UI
     */
    @TargetApi(Build.VERSION_CODES.HONEYCOMB_MR2)
    public void showProgress(final boolean show) {
        // On Honeycomb MR2 we have the ViewPropertyAnimator APIs, which allow
        // for very easy animations. If available, use these APIs to fade-in
        // the progress spinner.
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB_MR2) {
            int shortAnimTime = getResources().getInteger(android.R.integer.config_shortAnimTime);

            mVehicleFormView.setVisibility(show ? View.GONE : View.VISIBLE);
            mVehicleFormView.animate().setDuration(shortAnimTime).alpha(
                    show ? 0 : 1).setListener(new AnimatorListenerAdapter() {
                @Override
                public void onAnimationEnd(Animator animation) {
                    mVehicleFormView.setVisibility(show ? View.GONE : View.VISIBLE);
                }
            });

            mProgressView.setVisibility(show ? View.VISIBLE : View.GONE);
            mProgressView.animate().setDuration(shortAnimTime).alpha(
                    show ? 1 : 0).setListener(new AnimatorListenerAdapter() {
                @Override
                public void onAnimationEnd(Animator animation) {
                    mProgressView.setVisibility(show ? View.VISIBLE : View.GONE);
                }
            });
        } else {
            // The ViewPropertyAnimator APIs are not available, so simply show
            // and hide the relevant UI components.
            mProgressView.setVisibility(show ? View.VISIBLE : View.GONE);
            mVehicleFormView.setVisibility(show ? View.GONE : View.VISIBLE);
        }
    }

    @Override
    public void updateResources(Intervention intervention) {
        Log.i(TAG, "updateResources start");
        showProgress(false);
        if(intervention != null) {
            Log.i(TAG, "updateIntervention success");
            dismiss();
        }
    }
}

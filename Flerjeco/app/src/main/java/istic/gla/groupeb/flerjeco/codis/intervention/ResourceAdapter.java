package istic.gla.groupeb.flerjeco.codis.intervention;

import android.content.Context;
import android.graphics.drawable.Drawable;
import android.os.AsyncTask;
import android.support.v4.app.FragmentActivity;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import org.springframework.web.client.HttpStatusCodeException;

import java.util.List;

import entity.Intervention;
import entity.Resource;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.springRest.SpringService;

/**
 * Created by jules on 13/04/15.
 */
public class ResourceAdapter extends ArrayAdapter<Resource> {

    private List<Resource> resources;
    private long interventionId;
    private ResourcesFragment fragment;

    public ResourceAdapter(Context context, int resource, List<Resource> resources, Long interventionId, ResourcesFragment fragment) {
        super(context, resource, resources);
        this.resources = resources;
        this.interventionId = interventionId;
        this.fragment = fragment;
    }

    private static class ViewHolder {
        public TextView validationText;
        public Button acceptButton;
        public Button refuseButton;
        public long interventionId;
    }

    @Override
    public View getView(int position, View convertView, ViewGroup parent) {
        ViewHolder viewHolder;
        if (convertView == null) {
            convertView = LayoutInflater.from(this.getContext()).inflate(R.layout.item_resouce_codis, parent, false);
            // configure view holder
            viewHolder = new ViewHolder();
            viewHolder.validationText = (TextView) convertView.findViewById(R.id.text_validation_resource);
            viewHolder.acceptButton = (Button) convertView.findViewById(R.id.button_accept_resource);
            viewHolder.refuseButton = (Button) convertView.findViewById(R.id.button_refuse_resource);
            viewHolder.interventionId = interventionId;
            convertView.setTag(viewHolder);
        } else {
            viewHolder = (ViewHolder) convertView.getTag();
        }

        final Resource resource = resources.get(position);

        viewHolder.validationText.setText(resource.getLabel());

        final ViewHolder _viewHolder = viewHolder;
        viewHolder.acceptButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Toast.makeText(getContext(), "" + _viewHolder.interventionId + " - " + interventionId, Toast.LENGTH_LONG).show();
                if(_viewHolder.interventionId >= 0) {
                    new ResourceRequestTask().execute(
                            "" + interventionId,
                            resource.getLabel(),
                            fragment.getActivity().getString(R.string.state_waiting),
                            fragment.getActivity().getString(R.string.state_validated));
                }
            }
        });
        viewHolder.refuseButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if(interventionId >= 0)
                    new ResourceRequestTask().execute("" + interventionId,
                            resource.getLabel(),
                            fragment.getActivity().getString(R.string.state_waiting),
                            fragment.getActivity().getString(R.string.state_refused));
            }
        });

        return convertView;
    }
    private class ResourceRequestTask extends AsyncTask<String, Void, Intervention> {

        @Override
        protected Intervention doInBackground(String... params) {
            try {
                return new SpringService().changeResourceState(params);
            } catch (HttpStatusCodeException e) {
                Log.e("ResourceAdapterCodis", e.getMessage(), e);
            }

            return null;
        }

        @Override
        protected void onPostExecute(Intervention intervention) {
            fragment.updateResources(intervention);
            Log.i("ResourceAdapterCodis", "Request returned");
        }

    }
}
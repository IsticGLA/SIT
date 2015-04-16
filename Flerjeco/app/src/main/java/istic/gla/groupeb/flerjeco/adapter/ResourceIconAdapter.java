package istic.gla.groupeb.flerjeco.adapter;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;

import java.util.List;

import entity.Resource;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.icons.Danger;
import istic.gla.groupeb.flerjeco.icons.IIcon;
import istic.gla.groupeb.flerjeco.icons.Sensitive;
import istic.gla.groupeb.flerjeco.icons.Vehicle;
import istic.gla.groupeb.flerjeco.view.IconView;
import util.ResourceRole;

/**
 * Created by flolegazier on 13/04/15.
 */
public class ResourceIconAdapter extends ArrayAdapter<Resource> {

    private List<Resource> resources;

    private static class ViewHolder {
        public IconView iconViewResource;
    }

    public ResourceIconAdapter(Context context, int resource, List<Resource> resources) {
        super(context, resource, resources);
        this.resources = resources;
    }

    @Override
    public View getView(int position, View convertView, ViewGroup parent){
        IIcon mIcon = null;
        ViewHolder viewHolder;
        if (convertView == null){
            convertView = LayoutInflater.from(this.getContext()).inflate(R.layout.item_resource_agent_only_icon,parent,false);
            // configure view holder
            viewHolder = new ViewHolder();
            viewHolder.iconViewResource = (IconView) convertView.findViewById(R.id.image_view);
            convertView.setTag(viewHolder);
        }else{
            viewHolder = (ViewHolder) convertView.getTag();
        }

        Resource resource = resources.get(position);
        String label = resource.getLabel();
        Log.i("Adapter, resource : ",label);
        ResourceRole role = resource.getResourceRole() != null ? resource.getResourceRole() : ResourceRole.otherwise;

        switch (resource.getResourceCategory()){
            case vehicule:
                mIcon = new Vehicle(label, role, resource.getState());
                break;
            case dragabledata:
                if ("incident".equals(label)){
                    mIcon = new IIcon() {
                        @Override
                        public void drawIcon(Canvas mCanvas) {
                            Bitmap mBitmap = BitmapFactory.decodeResource(getContext().getResources(), R.drawable.incident);
                            mCanvas.drawBitmap(mBitmap,10,10,new Paint());
                        }
                    };
                } else if ("danger".equals(label)) {
                    mIcon = new Danger();
                } else if ("sensitive".equals(label)) {
                    mIcon = new Sensitive();
                }
        }
        if (mIcon != null) {
            viewHolder.iconViewResource.setIcon(mIcon);
        }

        viewHolder.iconViewResource.setResource(resource);

        return convertView;
    }
}

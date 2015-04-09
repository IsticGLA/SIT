package istic.gla.groupb.nivimoju.drone.latlong;

import org.apache.commons.lang3.StringUtils;
import org.apache.log4j.Logger;

/**
 * Classe de convertissage entre un système géodesique (latlong) et des coordonnées dans un repère local
 *
 * L'espace de travail est un rectangle dont les bords suivent les lignes de latitudes et longitudes (pas "en biais")
 */
public class LatLongConverter {
    Logger logger = Logger.getLogger(LatLongConverter.class);

    private final float latitudeTop;
    private final float latitudeBottom;
    private final float longitudeLeft;
    private final float longitudeRight;
    private final float width;
    private final float height;
    private final float offsetX;
    private final float offsetY;

    /**
     * Initialise un convertisseur latlong vers coordonnées locales dans un périmètre donné
     * Le point de coordonnées locales (0,0) est considéré comme étant au centre du rectangle déterminé par les latitudes et longitudes renseignées
     *
     * @param latitudeTop le bord supérieur du périmètre de travail
     * @param longitudeLeft le bord gauche du périmètre de travail
     * @param latitudeBottom le bord inférieur du périmètre de travail
     * @param longitudeRight le bord droit du périmètre de travail
     * @param width la dimension du bord gauche à droite en unités du repère local
     * @param height la dimension du bord inférieur à supérieur en unités du repère local
     */
    public LatLongConverter(float latitudeTop, float longitudeLeft, float latitudeBottom, float longitudeRight, float width, float height){
        this.latitudeTop = latitudeTop;
        this.longitudeLeft = longitudeLeft;
        this.latitudeBottom = latitudeBottom;
        this.longitudeRight = longitudeRight;
        this.width = width;
        this.height = height;
        this.offsetX = width / 2;
        this.offsetY = width / 2;
    }

    /**
     * Initialise un convertisseur latlong vers coordonnées locales dans un périmètre donné
     * Le point de coordonnée s locales (0,0) est considéré comme étant au centre du rectangle déterminé par les latitudes et longitudes renseignées
     *
     * @param topLeft le point en haut a gauche du périmètre de travail en coordonnées latlong
     * @param bottomRight le point en bas a droite du périmètre de travail en coordonnées latlong
     * @param width la dimension du bord gauche à droite en unités du repère local
     * @param height la dimension du bord inférieur à supérieur en unités du repère local
     */
    public LatLongConverter(LatLong topLeft, LatLong bottomRight, float width, float height){
        this(topLeft.getLatitude(), topLeft.getLongitude(), bottomRight.getLatitude(), bottomRight.getLongitude(), width, height);
    }

    /**
     * Retourne les coordonnées locales correspondantes à une coordonnée latlong
     * @param latlong les coordonnées à transformer
     * @return les coordonnées dans le système local
     * @throws java.lang.IllegalArgumentException si les coordonnées demandées sont en dehors du périmètre de travail
     */
    public LocalCoordinate getLocal(LatLong latlong) throws IllegalArgumentException{
        if(latlong.getLatitude() < latitudeBottom || latlong.getLatitude() > latitudeTop
                || latlong.getLongitude() < longitudeLeft ||latlong.getLongitude() > longitudeRight){
            logger.error("Les coordonnées sont en dehors du périmètre de travail, conversion impossible");
            throw  new IllegalArgumentException("Impossible de convertir une position hors du périmètre de travail défini");
        }
        //interpolation linéaire sur x puis y
        float x = width * (latlong.getLongitude() - longitudeLeft) / (longitudeRight - longitudeLeft) - offsetX;
        float y = height * (latlong.getLatitude() - latitudeBottom) / (latitudeTop - latitudeBottom) - offsetY;
        return new LocalCoordinate(x, y);
    }

    /**
     * Retourne les coordonnées latlong correspondantes à une coordonnée locale
     * @param local les coordonnées à transformer
     * @return les coordonnées dans le système latlong
     */
    public LatLong getLatLong(LocalCoordinate local){
        //interpolation linéaire sur x puis y
        float maxX = width - offsetX;
        float maxY = height - offsetY;
        float longitude = (longitudeRight - longitudeLeft) * (local.getX() - maxX) / (maxX - offsetX) + longitudeLeft;
        float latitude = (latitudeTop - latitudeBottom) * (local.getY() - maxY) / (maxY - offsetY) + latitudeBottom;
        return new LatLong(longitude,  latitude);
    }

    @Override
    public String toString() {
        return "LatLongConverter{" +
                "latitudeTop=" + latitudeTop +
                ", latitudeBottom=" + latitudeBottom +
                ", longitudeLeft=" + longitudeLeft +
                ", longitudeRight=" + longitudeRight +
                ", width=" + width +
                ", height=" + height +
                '}';
    }
}

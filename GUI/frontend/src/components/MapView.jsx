import { MapContainer, TileLayer, Marker, Popup } from "react-leaflet";
import "leaflet/dist/leaflet.css";
import { scoutIcon, sprayerIcon } from "./DroneIcons";
function DroneMap({ drones }) {
  return (
    <MapContainer
      center={drones.scout ? [drones.scout.lat, drones.scout.lon] : [29.9494229 , 76.8158501]}
      zoom={18}
      style={{ height: "500px", width: "100%" }}
    >
      <TileLayer
    url="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
    attribution="&copy; Esri"
  />

      {Object.entries(drones)?.map(([id, drone]) => (
        drone && (
          <Marker
            key={id}
            position={[drone.lat, drone.lon]}
            icon={id === "scout" ? scoutIcon : sprayerIcon}
          >
            <Popup>
              <strong>{id.toUpperCase()}</strong><br />
              Altitude: {drone.alt_m} m<br />
              Updated: {new Date(drone.timestamp * 1000).toLocaleTimeString()}
            </Popup>
          </Marker>
        )
      ))}
    </MapContainer>
  );
}
export default DroneMap;
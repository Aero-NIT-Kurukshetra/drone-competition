import express from "express";
import dotenv from "dotenv";
import RedisClient from "./redis/RedisClient.js";
import cors from "cors";
import multer from "multer";

const upload = multer();
dotenv.config();

const app = express();
app.use(cors());
app.use(express.json({ limit: "10mb" }));
let workers={"path_planner_worker":-1,"camera_worker":-1,"mission_manager":-1,"lidar_worker":-1}
let drone_data={"scout":{},"sprayer":{}};
let drone_gps={"scout":null,"sprayer":null};
const redis = new RedisClient({ workerId: "backend_server" });

(async () => {
  await redis.connect();
})();
function unixToReadable(ts) {
  return new Date(ts * 1000).toLocaleString("en-IN", {
    timeZone: "Asia/Kolkata"
  });
}

app.get("/", (_req, res) => {
  res.send("NIDAR-NITKKR Backend is running");
});
redis.listen("mission_manager:drone_battery_update", (data) => {
  const { drone_id } = data;
  // console.log("Battery update received for:", drone_id, data);
  if (drone_id === "scout") {
    drone_data["scout"] = data;
  } else if (drone_id === "sprayer") {
    drone_data["sprayer"] = data;
  }
});
redis.listen("mission_manager:drone_pose_update", (data) => {
  const { drone_id } = data;
  // console.log("Battery update received for:", drone_id, data);
  if (drone_id === "scout") {
    drone_gps["scout"] = data;
  } else if (drone_id === "sprayer") {
    drone_gps["sprayer"] = data;
  }
});

app.post("/api/kml/upload", upload.single("kml"), async (req, res) => {
  try {
    if (!req.file) {
      return res.status(400).json({ error: "No file uploaded" });
    }

    // Extract text from KML file
    const kmlText = req.file.buffer.toString("utf8");
    console.log(kmlText);


    await redis.publish(
      "mission_manager:scout_planning_request",
      {
        kml_xml: kmlText,   
        spacing: 5,
        angle: 60
      }
    );
    res.json({
      status: "uploaded"
    });
  } catch (err) {
    console.error("KML upload failed:", err);
    res.status(500).json({ error: "Internal server error" });
  }

});
app.get("/api/drones/gps", async (req, res) => {
  try {
    res.json({
      scout: drone_gps["scout"],
      sprayer: drone_gps["sprayer"]
    });
  } catch (err) {
    res.status(500).json({ error: "Failed to fetch GPS data" });
  }
});

app.get("/api/drone_data", async (req, res) => {
  try {
    res.json(drone_data);
  } catch (err) {
    console.error(err);
    res.status(500).json({ error: "Failed to fetch drone data" });
  }
});
app.get("/api/heartbeats", async (req, res) => {
  try {
    const keys = await redis.client.keys("heartbeat:*");
    
    for (const key of keys) {
      const value = await redis.client.get(key);
      if(value){
        const time=unixToReadable(JSON.parse(value).timestamp);
        workers[key.replace("heartbeat:", "")] = { ...JSON.parse(value), time: time };
      }
    }
    console.log(workers);
    res.json(workers);
  } catch (err) {
    console.error(err);
    res.status(500).json({ error: "Failed to fetch heartbeats" });
  }
});

// -------------------------------
// Health check
// -------------------------------
app.get("/health", async (_req, res) => {
  try {
    // await redis.client.ping();
    res.json({ status: "ok" });
  } catch {
    res.status(500).json({ status: "redis_down" });
  }
});

// -------------------------------
// Graceful shutdown
// -------------------------------
process.on("SIGINT", async () => {
  console.log("Shutting down...");
  await redis.close();
  process.exit(0);
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
  console.log(`Backend running on http://localhost:${PORT}`);
});

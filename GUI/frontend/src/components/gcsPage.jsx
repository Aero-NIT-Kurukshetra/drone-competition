import React, { useState, useEffect } from 'react';
import { Plus, Minus, Layers, Radio, Satellite, Cpu, CheckCircle, Clock, Loader } from 'lucide-react';
import { use } from 'react';
import DroneMap from './MapView';

const DroneGCSDashboard = () => {
  const [zoom, setZoom] = useState(1);
  const [components, setComponents] = useState([]);
  const [workers, setWorkers] = useState({});
  const [droneData, setDroneData] = useState({});
  const [droneGPS, setDroneGPS] = useState({});
  const addComponent = (type) => {
    const newComponent = {
      id: Date.now(),
      type,
      position: { x: 400, y: 250 }
    };
    setComponents([...components, newComponent]);
  };
  function fetchWorkers(url, intervalMs = 3000) {
    useEffect(() => {
      const timer = setInterval(async () => {
        try {
          const res = await fetch(url);
          const data = await res.json();
          setWorkers(data);

        } catch (err) {
          console.error(err);
        }
      }, intervalMs);

      return () => clearInterval(timer);
    }, [url, intervalMs]);
  }
  function fetchDroneData(url, intervalMs = 3000) {
    useEffect(() => {
      const timer = setInterval(async () => {
        try {
          const res = await fetch(url);
          const data = await res.json();
          // Process drone data as needed
          // For example, you can log it or update state
          console.log("Drone Data:", data);
          // console.log("Drone Data Scout Battery:", data.scout.remaining_pct);
          setDroneData(data);
        } catch (err) {
          console.error(err);
        }
      }, intervalMs);

      return () => clearInterval(timer);
    }, [url, intervalMs]);
  } 
  function fetchDroneGPS(url, intervalMs = 3000) {
    useEffect(() => {
      const timer = setInterval(async () => {
        try {
          const res = await fetch(url);
          const data = await res.json();
          // Process drone data as needed
          // For example, you can log it or update state
          console.log("Drone Data:", data);
          // console.log("Drone Data Scout Battery:", data.scout.remaining_pct);
          setDroneGPS(data);
        } catch (err) {
          console.error(err);
        }
      }, intervalMs);

      return () => clearInterval(timer);
    }, [url, intervalMs]);
  } 

  fetchWorkers('http://localhost:3000/api/heartbeats', 3000);
  fetchDroneData('http://localhost:3000/api/drone_data', 3000);
  fetchDroneGPS('http://localhost:3000/api/drones/gps', 3000);
  const removeComponent = (id) => {
    setComponents(components.filter(c => c.id !== id));
  };

  return (
    <div className="min-h-screen bg-gray-50 p-6">
      {/* Header */}
      <div className="mb-6">
        <div className="text-sm text-gray-500 mb-1 flex items-center gap-2">
          <div className="w-8 h-0.5 bg-blue-600"></div>
          COMMAND & CONTROL
        </div>
        <h1 className="text-4xl font-bold text-gray-900">
          Multi-Drone <span className="text-blue-600">GCS Dashboard</span>
        </h1>
      </div>

      <div className="grid grid-cols-12 gap-6">
        {/* Left Sidebar */}
        <div className="col-span-3 space-y-6">
          {/* Fleet Status */}
          <div className="bg-white rounded-lg shadow p-5">
            <div className="flex items-center justify-between mb-4">
              <div className="flex items-center gap-2 text-gray-700 font-semibold">
                <Radio className="w-4 h-4" />
                FLEET STATUS
              </div>
              <span className="px-2 py-1 bg-green-100 text-green-700 text-xs font-semibold rounded">LIVE</span>
            </div>

            <div className="space-y-4">
              <div className="border-l-4 border-blue-500 pl-3">
                <div className="flex justify-between items-start mb-1">
                  <div>
                    <div className="font-semibold text-gray-900">Scout-Drone</div>
                    <div className="text-xs text-gray-500">ID: VX-S1</div>
                  </div>
                  <div className="text-right">
                    <div className="text-xs text-gray-600 mb-1">Battery</div>
                    <div className="flex items-center gap-1">
                      <div className="w-2 h-2 bg-green-500 rounded-full"></div>
                      <span className="text-sm font-semibold">{droneData?.scout?.remaining_pct || "N/A"}%</span>
                    </div>
                  </div>
                </div>
                <div className="flex justify-between text-xs">
                  <span className="text-gray-600">Mode</span>
                  <span className="font-semibold text-gray-900">GUIDED</span>
                </div>
              </div>

              <div className="border-l-4 border-green-500 pl-3">
                <div className="flex justify-between items-start mb-1">
                  <div>
                    <div className="font-semibold text-gray-900">Sprayer-Drone</div>
                    <div className="text-xs text-gray-500">ID: VX-Y1</div>
                  </div>
                  <div className="text-right">
                    <div className="text-xs text-gray-600 mb-1">Battery</div>
                    <div className="flex items-center gap-1">
                      <div className="w-2 h-2 bg-green-500 rounded-full"></div>
                      <span className="text-sm font-semibold">{droneData.sprayer?.remaining_pct || "N/A"}%</span>
                    </div>
                  </div>
                </div>
                <div className="flex justify-between text-xs">
                  <span className="text-gray-600">Mode</span>
                  <span className="font-semibold text-gray-900">GUIDED</span>
                </div>
              </div>
            </div>
          </div>

          {/* Workers Section */}
          <div className="bg-white rounded-lg shadow p-5">
            <div className="flex items-center gap-2 text-gray-700 font-semibold mb-4">
              <Cpu className="w-4 h-4" />
              WORKERS
            </div>

            <div className="space-y-3">
              {Object.keys(workers).map(key => (

                <div key={key} className="border border-gray-200 rounded p-3">
                  <div className="flex justify-between items-center mb-2">
                    <div className="font-semibold text-gray-900 text-sm">{key}</div>
                    <div
                      className={`w-2 h-2 rounded-full ${workers[key]==-1 ? "bg-red-500" : "bg-green-500"
                        }`}
                    />

                  </div>
                  <div className="text-xs text-gray-600">Status: {workers[key] == -1 ? "Offline" : "Online"}</div>
                </div>

              ))}

            </div>
          </div>

          {/* Health Monitor */}
          <div className="bg-white rounded-lg shadow p-5">
            <div className="flex items-center gap-2 text-gray-700 font-semibold mb-4">
              <Radio className="w-4 h-4" />
              HEALTH MONITOR
            </div>

            <div className="space-y-4">
              <div>
                <div className="flex justify-between text-sm mb-1">
                  <span className="text-gray-600">Link Quality (Radio)</span>
                  <span className="font-semibold text-green-600">98%</span>
                </div>
                <div className="w-full bg-gray-200 rounded-full h-1.5">
                  <div className="bg-green-500 h-1.5 rounded-full" style={{ width: '98%' }}></div>
                </div>
              </div>

              <div>
                <div className="flex justify-between text-sm mb-1">
                  <span className="text-gray-600">GPS Satellites</span>
                  <span className="font-semibold text-blue-600">18 Sats (Locked)</span>
                </div>
                <div className="w-full bg-gray-200 rounded-full h-1.5">
                  <div className="bg-blue-500 h-1.5 rounded-full" style={{ width: '90%' }}></div>
                </div>
              </div>

              <div>
                <div className="flex justify-between text-sm mb-1">
                  <span className="text-gray-600">System CPU Load</span>
                  <span className="font-semibold text-gray-700">24%</span>
                </div>
                <div className="w-full bg-gray-200 rounded-full h-1.5">
                  <div className="bg-blue-500 h-1.5 rounded-full" style={{ width: '24%' }}></div>
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Center Map Area */}
        <div className="col-span-6">
          <div className="bg-white rounded-lg shadow h-full min-h-[600px] relative overflow-hidden">
            {/* Map Controls */}
            <div className="absolute top-4 left-4 z-10 flex gap-2">
              <button
                onClick={() => setZoom(z => Math.min(z + 0.2, 2))}
                className="bg-white shadow-md rounded p-2 hover:bg-gray-50"
              >
                <Plus className="w-5 h-5 text-gray-700" />
              </button>
              <button
                onClick={() => setZoom(z => Math.max(z - 0.2, 0.5))}
                className="bg-white shadow-md rounded p-2 hover:bg-gray-50"
              >
                <Minus className="w-5 h-5 text-gray-700" />
              </button>
              <button className="bg-white shadow-md rounded p-2 hover:bg-gray-50">
                <Layers className="w-5 h-5 text-gray-700" />
              </button>
            </div>

            {/* Map Content */}
            <div className="w-full h-full bg-gradient-to-br from-blue-50 to-gray-100 relative">
              {/* Flight Zone */}
              <DroneMap drones={droneGPS}/>
              </div>
          </div>
        </div>

        {/* Right Sidebar */}
        <div className="col-span-3 space-y-6">
          {/* Mission Queue */}
          <div className="bg-white rounded-lg shadow p-5">
            <div className="flex items-center gap-2 text-gray-700 font-semibold mb-4">
              <Layers className="w-4 h-4" />
              MISSION QUEUE
            </div>

            <div className="space-y-3">
              <div className="border-l-4 border-green-500 pl-3 py-2">
                <div className="flex justify-between items-start mb-1">
                  <div className="font-semibold text-gray-900 text-sm">Sector A Scan</div>
                  <div className="w-2 h-2 bg-green-500 rounded-full mt-1"></div>
                </div>
                <div className="text-xs text-gray-500">Completed 10:42 AM</div>
              </div>

              <div className="border-l-4 border-blue-500 pl-3 py-2">
                <div className="flex justify-between items-start mb-1">
                  <div className="font-semibold text-gray-900 text-sm">Process Imagery</div>
                  <Loader className="w-3 h-3 text-blue-500 animate-spin mt-1" />
                </div>
                <div className="text-xs text-blue-600 mb-1">ML Detection Running...</div>
                <div className="w-full bg-gray-200 rounded-full h-1.5">
                  <div className="bg-blue-500 h-1.5 rounded-full" style={{ width: '65%' }}></div>
                </div>
              </div>

              <div className="border-l-4 border-gray-300 pl-3 py-2">
                <div className="flex justify-between items-start mb-1">
                  <div className="font-semibold text-gray-900 text-sm">Deploy Sprayer</div>
                  <span className="text-xs text-gray-500 font-semibold">PENDING</span>
                </div>
                <div className="text-xs text-gray-500">Waiting for previous task...</div>
              </div>
            </div>
          </div>

          {/* Add Component */}
          <div className="bg-white rounded-lg shadow p-5">
            <div className="flex items-center gap-2 text-gray-700 font-semibold mb-4">
              <Plus className="w-4 h-4" />
              ADD COMPONENT
            </div>
            <div className="space-y-3">
              <button
                onClick={() => addComponent('camera')}
                className="w-full bg-blue-600 text-white py-2 rounded hover:bg-blue-700"
              >
                Add Camera Module
              </button>
              <button
                onClick={() => addComponent('sensor')}
                className="w-full bg-purple-600 text-white py-2 rounded hover:bg-purple-700"
              >
                Add Sensor Module
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
export default DroneGCSDashboard;
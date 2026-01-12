import React, { useState } from 'react';
import { Wifi, MapPin, Cpu, Droplets, CheckCircle, Upload, Play } from 'lucide-react';
import fs from "fs";

// import { parseKmlText } from '../worker/readkml.js';
const DeploymentWorkflow = () => {
  const [kmlFile, setKmlFile] = useState(null);
  const [activeTab, setActiveTab] = useState('operator');

  // Sample data - these would come from backend
  const [workflowData, setWorkflowData] = useState({
    setup: {
      tasks: ['Upload KML', 'Start Mission']
    },
    scouting: {
      drone: 'Drone 1 (Scout)',
      tasks: ['Lawnmower Path', 'RGB Mapping']
    },
    detection: {
      model: 'HSV model',
      tasks: ['Disease detection by OpenCV', 'Targeted crops marked for sprayer drone']
    },
    action: {
      drone: 'Drone 2 (sprayer)',
      tasks: ['Path Planning', 'Spot Spray']
    },
    finish: {
      tasks: ['Return to Launch', 'Log Save']
    }
  });

  const [timeline, setTimeline] = useState({
    scout: { label: 'Scout Drone', duration: 10, color: 'bg-emerald-400', status: 'Scanning' },
    processing: { label: 'Processing', duration: 10, color: 'bg-indigo-400', status: 'Compute' },
    spray: { label: 'Spray Drone', duration: 20, color: 'bg-blue-500', status: 'Treatment' }
  });

  // const handleFileUpload = async (event) => {
  //   const file = event.target.files[0];
  //   if (file) {
  //     setKmlFile(file);
  //     const { rawText, parsed } = await parseKmlText(file.path);
  //     const res=fetch('http://localhost:3000/api/kml',{
  //       method:'POST',
  //       headers:{
  //         'Content-Type':'application/json'
  //       },
  //       body:JSON.stringify({kmlText:rawText})
  //     });
  //     console.log('KML file uploaded:', file.name);
  //     console.log('Server response:', res);
  //   }
  // };
  function handleFileUpload(event) {
    const file = event.target.files[0];
    if (file) {
      setKmlFile(file);
      console.log("Selected KML file:", file.name);
    }
  }
  function handleStartMission(event) {
    
  const file = kmlFile;
  if (!file) return;
  const formData = new FormData();
  formData.append("kml", file);
  setKmlFile(file);
  console.log("Uploading KML file:", file.name);
  fetch("http://localhost:3000/api/kml/upload", {
    method: "POST",
    body: formData
  })
    .then(res => res.json())
    .then(data => {
      console.log("Upload response:", data);
    })
    .catch(err => {
      console.error("Upload failed:", err);
    });
}

  return (
    <div className="min-h-screen bg-gray-50 p-8">
      <div className="max-w-7xl mx-auto">
        {/* Header */}
        <div className="mb-8">
          <div className="text-teal-600 text-xs font-semibold tracking-wider mb-2">
            — OPERATIONAL SEQUENCE
          </div>
          <h1 className="text-4xl font-bold text-gray-900">
            Deployment & <span className="text-blue-600">Mission Workflow</span>
          </h1>
        </div>

        {/* Tabs */}
        <div className="flex gap-8 mb-8 border-b-2 border-gray-200">
          <button
            onClick={() => setActiveTab('operator')}
            className={`pb-3 px-1 text-sm font-semibold tracking-wide transition-colors ${
              activeTab === 'operator'
                ? 'text-blue-600 border-b-2 border-blue-600 -mb-0.5'
                : 'text-gray-500'
            }`}
          >
            OPERATOR PHASE
          </button>
          <button
            onClick={() => setActiveTab('autonomous')}
            className={`pb-3 px-1 text-sm font-semibold tracking-wide transition-colors ${
              activeTab === 'autonomous'
                ? 'text-emerald-500 border-b-2 border-emerald-500 -mb-0.5'
                : 'text-gray-500'
            }`}
          >
            AUTONOMOUS PHASE
          </button>
        </div>

        {/* Workflow Steps */}
        <div className="grid grid-cols-5 gap-4 mb-8">
          {/* Step 1: Setup */}
          <div className="bg-white rounded-lg shadow-sm border border-gray-200 p-6 relative">
            <div className="absolute -right-2 top-1/2 transform -translate-y-1/2 text-gray-300">
              <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
                <path d="M6 4l4 4-4 4V4z" />
              </svg>
            </div>
            <div className="flex justify-center mb-4">
              <div className="w-12 h-12 bg-blue-50 rounded-lg flex items-center justify-center">
                <Wifi className="w-6 h-6 text-blue-600" />
              </div>
            </div>
            <h3 className="text-center text-lg font-bold text-gray-900 mb-1">1. Setup</h3>
            <p className="text-center text-xs text-gray-500 mb-4">Manual Input</p>
            <div className="space-y-2">
              {workflowData.setup.tasks.map((task, idx) => (
                <div key={idx} className="flex items-start gap-2 text-xs text-gray-700">
                  <svg className="w-3 h-3 text-gray-400 mt-0.5 flex-shrink-0" fill="currentColor" viewBox="0 0 16 16">
                    <path d="M13.854 3.646a.5.5 0 0 1 0 .708l-7 7a.5.5 0 0 1-.708 0l-3.5-3.5a.5.5 0 1 1 .708-.708L6.5 10.293l6.646-6.647a.5.5 0 0 1 .708 0z"/>
                  </svg>
                  <span>{task}</span>
                </div>
              ))}
            </div>
          </div>

          {/* Step 2: Scouting */}
          <div className="bg-white rounded-lg shadow-sm border border-gray-200 p-6 relative">
            <div className="absolute -right-2 top-1/2 transform -translate-y-1/2 text-gray-300">
              <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
                <path d="M6 4l4 4-4 4V4z" />
              </svg>
            </div>
            <div className="flex justify-center mb-4">
              <div className="w-12 h-12 bg-emerald-50 rounded-lg flex items-center justify-center">
                <MapPin className="w-6 h-6 text-emerald-600" />
              </div>
            </div>
            <h3 className="text-center text-lg font-bold text-gray-900 mb-1">2. Scouting</h3>
            <p className="text-center text-xs text-gray-500 mb-4">{workflowData.scouting.drone}</p>
            <div className="space-y-2">
              {workflowData.scouting.tasks.map((task, idx) => (
                <div key={idx} className="flex items-start gap-2 text-xs text-gray-700">
                  <svg className="w-3 h-3 text-gray-400 mt-0.5 flex-shrink-0" fill="currentColor" viewBox="0 0 16 16">
                    <path d="M13.854 3.646a.5.5 0 0 1 0 .708l-7 7a.5.5 0 0 1-.708 0l-3.5-3.5a.5.5 0 1 1 .708-.708L6.5 10.293l6.646-6.647a.5.5 0 0 1 .708 0z"/>
                  </svg>
                  <span>{task}</span>
                </div>
              ))}
            </div>
          </div>

          {/* Step 3: Detection */}
          <div className="bg-white rounded-lg shadow-sm border border-gray-200 p-6 relative">
            <div className="absolute -right-2 top-1/2 transform -translate-y-1/2 text-gray-300">
              <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
                <path d="M6 4l4 4-4 4V4z" />
              </svg>
            </div>
            <div className="flex justify-center mb-4">
              <div className="w-12 h-12 bg-teal-50 rounded-lg flex items-center justify-center">
                <Cpu className="w-6 h-6 text-teal-600" />
              </div>
            </div>
            <h3 className="text-center text-lg font-bold text-gray-900 mb-1">3. Detection</h3>
            <p className="text-center text-xs text-gray-500 mb-4">{workflowData.detection.model}</p>
            <div className="space-y-2">
              {workflowData.detection.tasks.map((task, idx) => (
                <div key={idx} className="flex items-start gap-2 text-xs text-gray-700">
                  <svg className="w-3 h-3 text-gray-400 mt-0.5 flex-shrink-0" fill="currentColor" viewBox="0 0 16 16">
                    <path d="M13.854 3.646a.5.5 0 0 1 0 .708l-7 7a.5.5 0 0 1-.708 0l-3.5-3.5a.5.5 0 1 1 .708-.708L6.5 10.293l6.646-6.647a.5.5 0 0 1 .708 0z"/>
                  </svg>
                  <span>{task}</span>
                </div>
              ))}
            </div>
          </div>

          {/* Step 4: Action */}
          <div className="bg-white rounded-lg shadow-sm border border-gray-200 p-6 relative">
            <div className="absolute -right-2 top-1/2 transform -translate-y-1/2 text-gray-300">
              <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
                <path d="M6 4l4 4-4 4V4z" />
              </svg>
            </div>
            <div className="flex justify-center mb-4">
              <div className="w-12 h-12 bg-emerald-50 rounded-lg flex items-center justify-center">
                <Droplets className="w-6 h-6 text-emerald-600" />
              </div>
            </div>
            <h3 className="text-center text-lg font-bold text-gray-900 mb-1">4. Action</h3>
            <p className="text-center text-xs text-gray-500 mb-4">{workflowData.action.drone}</p>
            <div className="space-y-2">
              {workflowData.action.tasks.map((task, idx) => (
                <div key={idx} className="flex items-start gap-2 text-xs text-gray-700">
                  <svg className="w-3 h-3 text-gray-400 mt-0.5 flex-shrink-0" fill="currentColor" viewBox="0 0 16 16">
                    <path d="M13.854 3.646a.5.5 0 0 1 0 .708l-7 7a.5.5 0 0 1-.708 0l-3.5-3.5a.5.5 0 1 1 .708-.708L6.5 10.293l6.646-6.647a.5.5 0 0 1 .708 0z"/>
                  </svg>
                  <span>{task}</span>
                </div>
              ))}
            </div>
          </div>

          {/* Step 5: Finish */}
          <div className="bg-white rounded-lg shadow-sm border border-gray-200 p-6">
            <div className="flex justify-center mb-4">
              <div className="w-12 h-12 bg-teal-50 rounded-lg flex items-center justify-center">
                <CheckCircle className="w-6 h-6 text-teal-600" />
              </div>
            </div>
            <h3 className="text-center text-lg font-bold text-gray-900 mb-1">5. Finish</h3>
            <p className="text-center text-xs text-gray-500 mb-4">Mission End</p>
            <div className="space-y-2">
              {workflowData.finish.tasks.map((task, idx) => (
                <div key={idx} className="flex items-start gap-2 text-xs text-gray-700">
                  <svg className="w-3 h-3 text-gray-400 mt-0.5 flex-shrink-0" fill="currentColor" viewBox="0 0 16 16">
                    <path d="M13.854 3.646a.5.5 0 0 1 0 .708l-7 7a.5.5 0 0 1-.708 0l-3.5-3.5a.5.5 0 1 1 .708-.708L6.5 10.293l6.646-6.647a.5.5 0 0 1 .708 0z"/>
                  </svg>
                  <span>{task}</span>
                </div>
              ))}
            </div>
          </div>
        </div>

        {/* Bottom Section */}
        <div className="grid grid-cols-2 gap-6">
          {/* Left: Minimal Operator Load */}
          <div className="bg-white rounded-lg shadow-sm border border-gray-200 p-6">
            <div className="flex items-center gap-2 mb-6">
              <div className="w-8 h-8 bg-blue-50 rounded flex items-center justify-center">
                <svg className="w-4 h-4 text-blue-600" fill="currentColor" viewBox="0 0 16 16">
                  <path d="M8 8a3 3 0 1 0 0-6 3 3 0 0 0 0 6zm2-3a2 2 0 1 1-4 0 2 2 0 0 1 4 0zm4 8c0 1-1 1-1 1H3s-1 0-1-1 1-4 6-4 6 3 6 4zm-1-.004c-.001-.246-.154-.986-.832-1.664C11.516 10.68 10.289 10 8 10c-2.29 0-3.516.68-4.168 1.332-.678.678-.83 1.418-.832 1.664h10z"/>
                </svg>
              </div>
              <h3 className="text-sm font-bold text-gray-900">Minimal Operator Load</h3>
            </div>

            <div className="mb-6">
              <div className="flex items-center justify-between mb-3">
                <span className="text-xs text-gray-600">GCS_v2.0</span>
                <div className="w-3 h-3 bg-teal-500 rounded-full"></div>
              </div>
              
              <div className="bg-blue-50 border-2 border-blue-200 rounded-lg p-4 mb-4">
                <div className="flex items-center justify-center gap-2 text-blue-600 text-sm font-semibold mb-3">
                  <Upload className="w-4 h-4" />
                  <span>KML Uploaded</span>
                </div>
                {kmlFile && (
                  <div className="text-xs text-blue-700 text-center">
                    {kmlFile.name}
                  </div>
                )}
              </div>

              <input
                type="file"
                accept=".kml"
                onChange={handleFileUpload}
                className="hidden"
                id="kml-upload"
              />
              <label
                htmlFor="kml-upload"
                className="block w-full bg-gray-100 hover:bg-gray-200 text-gray-700 text-sm font-medium py-2 px-4 rounded-lg cursor-pointer text-center transition-colors mb-3"
              >
                Choose KML File
              </label>

              <button
                onClick={handleStartMission}
                disabled={!kmlFile}
                className="w-full bg-blue-600 hover:bg-blue-700 disabled:bg-blue-300 text-white font-semibold py-3 px-4 rounded-lg flex items-center justify-center gap-2 transition-colors"
              >
                <Play className="w-4 h-4" />
                START MISSION
              </button>
            </div>

            <p className="text-xs text-gray-400 text-center italic">
              Operator role ends after click
            </p>
          </div>

          {/* Right: Autonomous Execution Timeline */}
          <div className="bg-white rounded-lg shadow-sm border border-gray-200 p-6">
            <div className="flex items-center gap-2 mb-6">
              <div className="w-8 h-8 bg-emerald-50 rounded flex items-center justify-center">
                <svg className="w-4 h-4 text-emerald-600" fill="currentColor" viewBox="0 0 16 16">
                  <rect x="2" y="2" width="3" height="3" />
                  <rect x="6" y="2" width="3" height="3" />
                  <rect x="10" y="2" width="3" height="3" />
                  <rect x="2" y="6" width="3" height="3" />
                  <rect x="6" y="6" width="3" height="3" />
                  <rect x="10" y="6" width="3" height="3" />
                  <rect x="2" y="10" width="3" height="3" />
                  <rect x="6" y="10" width="3" height="3" />
                  <rect x="10" y="10" width="3" height="3" />
                </svg>
              </div>
              <h3 className="text-sm font-bold text-gray-900">Autonomous Execution Timeline</h3>
            </div>

            <div className="space-y-5">
              {/* Scout Drone */}
              <div>
                <div className="flex items-center justify-between mb-2">
                  <span className="text-xs text-gray-600 font-medium">Scout Drone</span>
                  <span className="text-xs text-gray-500">{timeline.scout.status}</span>
                </div>
                <div className="relative h-2 bg-gray-200 rounded-full overflow-hidden">
                  <div className={`absolute left-0 top-0 h-full ${timeline.scout.color} rounded-full`} style={{width: '100%'}}></div>
                </div>
              </div>

              {/* Processing */}
              <div>
                <div className="flex items-center justify-between mb-2">
                  <span className="text-xs text-gray-600 font-medium">Processing</span>
                  <span className="text-xs text-gray-500">{timeline.processing.status}</span>
                </div>
                <div className="relative h-2 bg-gray-200 rounded-full overflow-hidden">
                  <div className={`absolute left-0 top-0 h-full ${timeline.processing.color} rounded-full`} style={{width: '100%'}}></div>
                </div>
              </div>

              {/* Spray Drone */}
              <div>
                <div className="flex items-center justify-between mb-2">
                  <span className="text-xs text-gray-600 font-medium">Spray Drone</span>
                  <span className="text-xs text-gray-500">{timeline.spray.status}</span>
                </div>
                <div className="relative h-2 bg-gray-200 rounded-full overflow-hidden">
                  <div className={`absolute left-0 top-0 h-full ${timeline.spray.color} rounded-full`} style={{width: '100%'}}></div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default DeploymentWorkflow;
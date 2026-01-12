import { BrowserRouter, Routes, Route } from "react-router-dom";
import DeploymentWorkflow from "./components/setup.jsx";
import DroneGCSDashboard from "./components/gcsPage.jsx";
function App() {
  return (
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<DeploymentWorkflow />} />
        <Route path="/gcs" element={<DroneGCSDashboard />} />
      </Routes>
    </BrowserRouter>
  );
}

export default App;

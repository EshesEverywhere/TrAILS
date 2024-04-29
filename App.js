import React from "react";
import { BrowserRouter as Router, Routes, Route } from "react-router-dom";
import "./App.css";
// import Navbar from "./components/Navbar";
import Map from "./components/Map";
import SOS from "./components/SOSComponent";

function App() {
  const [data, setData] = React.useState(null);

  return (
    <>
      <Router>
        {/* <Navbar /> */}
        <Routes>
          <Route path="/" exact />
        </Routes>
      </Router>
      <div className="components-container">
        <SOS />
        <Map />
      </div>
      {/* <p>{!data ? "Loading..." : data}</p> */}
    </>
  );
}

export default App;

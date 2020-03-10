import ReactDOM from "react-dom";
import React, { useState } from "react";
import App from "./App";
import 'bootstrap/dist/css/bootstrap.min.css';

// Connecting to server

function render(content) {
  ReactDOM.render(content, document.getElementById("root"));
}

render(
  <div className="App" style={{ width: "100vw", height: "100vh" }}>
    <App />
  </div>
);

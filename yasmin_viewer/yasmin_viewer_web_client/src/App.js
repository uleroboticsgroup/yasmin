import React, { Fragment } from "react";
import "./App.css";
import Viewer from "./components/Viewer";

function App() {
  return (
    <Fragment>
      <div
        style={{
          marginTop: "20px",
          marginLeft: "20px",
          marginRight: "20px",
          marginBottom: "20px",
        }}
      >
        <Viewer />
      </div>
    </Fragment>
  );
}

export default App;

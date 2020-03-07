import { useAnimationFrame } from "@cruise-automation/hooks";

import React, { useState } from "react";
import ROSLIB from "roslib";
import Worldview, { Cubes, Grid, Axes } from "regl-worldview";

function Example() {
  // position
  var xpos = 0;
  var ypos = 0;

  // Connecting to ROS websockets
  var ros = new ROSLIB.Ros({
    url: "ws://robot.local:9090"
  });

  ros.on("connection", function() {
    console.log("Connected to websocket server.");

    // create a listener
    var listener = new ROSLIB.Topic({
      ros: ros,
      name: "/dwm1001/tag",
      messageType: "localizer_dwm1001/Tag"
    });

    // subscribe to Tag topic
    listener.subscribe(function(message) {
      xpos = message.x;
      ypos = message.y;
      console.log("Received message on " + listener.name + ": " + xpos);

      // listener.unsubscribe();
    });
  });

  ros.on("error", function(error) {
    console.log("Error connecting to websocket server: ", error);
  });

  ros.on("close", function() {
    console.log("Connection to websocket server closed.");
  });

  const [count, setCount] = useState(0);
  const validatedCount = isNaN(count) || count < 1 ? 6 : Math.round(count);
  useAnimationFrame(
    () => {
      // update count before each browser repaint
      const newCount = count + 1;
      setCount(newCount);
    },
    false,
    []
  );
  return (
    <Worldview>
      <Grid count={100} />
      <Axes />

      <Cubes>
        {[
          {
            pose: {
              orientation: { x: 0, y: 0, z: 0, w: 1 },
              // position the cube at the center
              position: { x: xpos, y: ypos, z: 0 }
            },
            scale: { x: 5, y: 10, z: 5 },
            // rgba values are between 0 and 1 (inclusive)
            color: { r: 1, g: 0, b: 0, a: 1 }
          }
        ]}
      </Cubes>

      <div
        style={{
          position: "absolute",
          display: "flex",
          flexDirection: "column",
          padding: 8,
          left: 0,
          top: 0,
          right: 0,
          maxWidth: "100%",
          color: "white",
          backgroundColor: "rgba(0, 0, 0, 0.5)"
        }}
      >
        <div>
          Set grid count:
          <input
            style={{ width: 32, marginLeft: 8 }}
            value={count}
            onChange={event => setCount(Number(event.target.value))}
          />
        </div>
      </div>
    </Worldview>
  );
}

export default Example;

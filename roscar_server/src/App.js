import { useAnimationFrame } from "@cruise-automation/hooks";

import React, { useState } from "react";
import ROSLIB from "roslib";
import Worldview, { Text, Lines, Cubes, Grid, Axes } from "regl-worldview";

let anchor0pos = { id: "anchor0", x: 0, y: 0 };
let anchor1pos = { id: "anchor1", x: 10, y: 0 };
let anchor2pos = { id: "anchor2", x: 10, y: 10 };
let anchor3pos = { id: "anchor3", x: 0, y: 10 };
let tagpos = { x: 5, y: 5 };
let tagOrientation = {x: 0, y: 0, z: 0, w: 0}

function App() {
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

  // lines
  const lines = [
    {
      pose: {
        position: { x: 0, y: 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 0 }
      },
      scale: { x: 0.1, y: 0.1, z: 0.1 },
      color: { r: 0, g: 1, b: 0, a: 1 },
      points: [
        { x: anchor0pos.x, y: anchor0pos.y, z: 0 },
        { x: tagpos.x, y: tagpos.y, z: 0 }
      ],
      colors: []
    },
    {
      pose: {
        position: { x: 0, y: 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 0 }
      },
      scale: { x: 0.1, y: 0.1, z: 0.1 },
      color: { r: 0, g: 1, b: 0, a: 1 },
      points: [
        { x: anchor1pos.x, y: anchor1pos.y, z: 0 },
        { x: tagpos.x, y: tagpos.y, z: 0 }
      ],
      colors: []
    },
    {
      pose: {
        position: { x: 0, y: 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 0 }
      },
      scale: { x: 0.1, y: 0.1, z: 0.1 },
      color: { r: 0, g: 1, b: 0, a: 1 },
      points: [
        { x: anchor2pos.x, y: anchor2pos.y, z: 0 },
        { x: tagpos.x, y: tagpos.y, z: 0 }
      ],
      colors: []
    },
    {
      pose: {
        position: { x: 0, y: 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 0 }
      },
      scale: { x: 0.1, y: 0.1, z: 0.1 },
      color: { r: 0, g: 1, b: 0, a: 1 },
      points: [
        { x: anchor3pos.x, y: anchor3pos.y, z: 0 },
        { x: tagpos.x, y: tagpos.y, z: 0 }
      ],
      colors: []
    }
  ];

  const labelMarkers = [
    {
      name: anchor0pos.id,
      text: anchor0pos.id,
      color: { r: 1, g: 1, b: 1, a: 1 },
      pose: {
        orientation: { x: 0, y: 0, z: 0, w: 1 },
        position: { x: anchor0pos.x, y: anchor0pos.y, z: 0, w: 1 }
      },
      scale: { x: 1, y: 1, z: 1 }
      // uncomment colors and remove autoBackgroundColor prop to set text and background colors
      // colors: [{ r: 1, g: 1, b: 1, a: 1 }, { r: 1, g: 0, b: 0, a: 0.8 }],
    },
    {
      name: anchor1pos.id,
      text: anchor1pos.id,
      color: { r: 1, g: 1, b: 1, a: 1 },
      pose: {
        orientation: { x: 0, y: 0, z: 0, w: 1 },
        position: { x: anchor1pos.x, y: anchor1pos.y, z: 0, w: 1 }
      },
      scale: { x: 1, y: 1, z: 1 }
      // uncomment colors and remove autoBackgroundColor prop to set text and background colors
      // colors: [{ r: 1, g: 1, b: 1, a: 1 }, { r: 1, g: 0, b: 0, a: 0.8 }],
    },
    {
      name: anchor2pos.id,
      text: anchor2pos.id,
      color: { r: 1, g: 1, b: 1, a: 1 },
      pose: {
        orientation: { x: 0, y: 0, z: 0, w: 1 },
        position: { x: anchor2pos.x, y: anchor2pos.y, z: 0, w: 1 }
      },
      scale: { x: 1, y: 1, z: 1 }
      // uncomment colors and remove autoBackgroundColor prop to set text and background colors
      // colors: [{ r: 1, g: 1, b: 1, a: 1 }, { r: 1, g: 0, b: 0, a: 0.8 }],
    },
    {
      name: anchor3pos.id,
      text: anchor3pos.id,
      color: { r: 1, g: 1, b: 1, a: 1 },
      pose: {
        orientation: { x: 0, y: 0, z: 0, w: 1 },
        position: { x: anchor3pos.x, y: anchor3pos.y, z: 0, w: 1 }
      },
      scale: { x: 1, y: 1, z: 1 }
      // uncomment colors and remove autoBackgroundColor prop to set text and background colors
      // colors: [{ r: 1, g: 1, b: 1, a: 1 }, { r: 1, g: 0, b: 0, a: 0.8 }],
    }
  ];

  return (
    <Worldview>
      <Grid count={100} />
      <Axes />
      <Lines>{lines}</Lines>
      <Text>{labelMarkers}</Text>
      <Cubes>
        {[
          {
            pose: {
              orientation: { x: 0, y: 0, z: tagOrientation.z, w: tagOrientation.w },
              // position the cube at the center
              position: { x: tagpos.x, y: tagpos.y, z: 0 }
            },
            scale: { x: 1, y: 1, z: 1 },
            // rgba values are between 0 and 1 (inclusive)
            color: { r: 1, g: 0, b: 0, a: 1 }
          }
        ]}
      </Cubes>

      <Cubes>
        {[
          {
            pose: {
              orientation: { x: 0, y: 0, z: 0, w: 1 },
              // position the cube at the center
              position: { x: anchor0pos.x, y: anchor0pos.y, z: 0 }
            },
            scale: { x: 1, y: 1, z: 1 },
            // rgba values are between 0 and 1 (inclusive)
            color: { r: 0, g: 1, b: 0, a: 1 }
          }
        ]}
      </Cubes>
      <Cubes>
        {[
          {
            pose: {
              orientation: { x: 0, y: 0, z: 0, w: 1 },
              // position the cube at the center
              position: { x: anchor1pos.x, y: anchor1pos.y, z: 0 }
            },
            scale: { x: 1, y: 1, z: 1 },
            // rgba values are between 0 and 1 (inclusive)
            color: { r: 0, g: 1, b: 0, a: 1 }
          }
        ]}
      </Cubes>
      <Cubes>
        {[
          {
            pose: {
              orientation: { x: 0, y: 0, z: 0, w: 1 },
              // position the cube at the center
              position: { x: anchor2pos.x, y: anchor2pos.y, z: 0 }
            },
            scale: { x: 1, y: 1, z: 1 },
            // rgba values are between 0 and 1 (inclusive)
            color: { r: 0, g: 1, b: 0, a: 1 }
          }
        ]}
      </Cubes>
      <Cubes>
        {[
          {
            pose: {
              orientation: { x: 0, y: 0, z: 0, w: 1 },
              // position the cube at the center
              position: { x: anchor3pos.x, y: anchor3pos.y, z: 0 }
            },
            scale: { x: 1, y: 1, z: 1 },
            // rgba values are between 0 and 1 (inclusive)
            color: { r: 0, g: 1, b: 0, a: 1 }
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
          backgroundColor: "rgba(0.5, 0, 0, 0.5)"
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

function ConnectROSbridge() {
  // Connecting to ROS websockets
  var ros = new ROSLIB.Ros({
    url: "ws://robot.local:9090"
  });

  ros.on("connection", function() {
    console.log("Connected to websocket server.");

    // create a listener
    var Anchor0Listener = new ROSLIB.Topic({
      ros: ros,
      name: "/dwm1001/anchor0",
      messageType: "localizer_dwm1001/Anchor"
    });
    var Anchor1Listener = new ROSLIB.Topic({
      ros: ros,
      name: "/dwm1001/anchor1",
      messageType: "localizer_dwm1001/Anchor"
    });
    var Anchor2Listener = new ROSLIB.Topic({
      ros: ros,
      name: "/dwm1001/anchor2",
      messageType: "localizer_dwm1001/Anchor"
    });
    var Anchor3Listener = new ROSLIB.Topic({
      ros: ros,
      name: "/dwm1001/anchor3",
      messageType: "localizer_dwm1001/Anchor"
    });
    var tagListener = new ROSLIB.Topic({
      ros: ros,
      name: "/dwm1001/tag",
      messageType: "localizer_dwm1001/Tag"
    });

    var IMUlistener = new ROSLIB.Topic({
      ros: ros,
      name: "/imu/data",
      messageType: "sensor_msgs/Imu"
    });

    IMUlistener.subscribe(function(message) {
      console.log("IMU: " + message.orientation.x);
      tagOrientation.x = message.orientation.x;
      tagOrientation.y = message.orientation.y;
      tagOrientation.z = message.orientation.z;
      tagOrientation.w = message.orientation.w;

    });

    Anchor0Listener.subscribe(function(message) {
      anchor0pos.x = 10 * message.x;
      anchor0pos.y = 10 * message.y;
      // listener.unsubscribe();
    });

    Anchor1Listener.subscribe(function(message) {
      anchor1pos.x = 10 * message.x;
      anchor1pos.y = 10 * message.y;
      // listener.unsubscribe();
    });

    Anchor2Listener.subscribe(function(message) {
      anchor2pos.x = 10 * message.x;
      anchor2pos.y = 10 * message.y;
      // listener.unsubscribe();
    });

    Anchor3Listener.subscribe(function(message) {
      anchor3pos.x = 10 * message.x;
      anchor3pos.y = 10 * message.y;
      // listener.unsubscribe();
    });

    // subscribe to Tag topic
    tagListener.subscribe(function(message) {
      tagpos.x = 10 * message.x;
      tagpos.y = 10 * message.y;
      // console.log(
      //   "Received message on " +
      //     tagListener.name +
      //     ": " +
      //     tagpos.x +
      //     ", " +
      //     tagpos.y
      // );

      // listener.unsubscribe();
    });
  });

  ros.on("error", function(error) {
    console.log("Error connecting to websocket server: ", error);
  });

  ros.on("close", function() {
    console.log("Connection to websocket server closed.");
  });
}
ConnectROSbridge();
export default App;
